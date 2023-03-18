/****************************************************************************
 * MeshLab                                                           o o     *
 * A versatile mesh processing toolbox                             o     o   *
 *                                                                _   O  _   *
 * Copyright(C) 2005                                                \/)\/    *
 * Visual Computing Lab                                            /\/|      *
 * ISTI - Italian National Research Council                           |      *
 *                                                                    \      *
 * All rights reserved.                                                      *
 *                                                                           *
 * This program is free software; you can redistribute it and/or modify      *
 * it under the terms of the GNU General Public License as published by      *
 * the Free Software Foundation; either version 2 of the License, or         *
 * (at your option) any later version.                                       *
 *                                                                           *
 * This program is distributed in the hope that it will be useful,           *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 * GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
 * for more details.                                                         *
 *                                                                           *
 ****************************************************************************/

#include "PolylineCuttingFilter.h"

#include "common/PolylineMesh.h"
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/curve_on_manifold.h>
#include <vcg/complex/algorithms/outline_support.h>
#include <vcg/space/fitting3.h>

namespace curvatureSkeleton
{

static void movePolylineToFittingPlane(PolylineMesh& polyline, vcg::Plane3<Scalarm> const& plane, Scalarm weight);
static void movePolylinesApart(PolylineMesh& lpolyline, PolylineMesh const& rpolyline, Scalarm min_distance, Scalarm weight);

std::map<std::string, QVariant> PolylineCuttingFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos* cb)
{
	//parameters
	auto* original_mm = document.mm();
	auto& original = original_mm->cm;
	auto facetag_id = params.getString(PARAM_FACE_TAG_ID);

	//refine parameters
	auto num_iter = params.getInt(PARAM_REFINE_ITERATIONS); if (num_iter < 0) { num_iter = 0; }
	auto smooth_w = params.getDynamicFloat(PARAM_REFINE_SMOOTH_WEIGTH);
	auto proj_w = params.getDynamicFloat(PARAM_REFINE_PROJECT_WEIGTH);
	auto fit_plane_w = params.getDynamicFloat(PARAM_REFINE_FIT_PLANE_WEIGTH);
	auto separation_w = params.getDynamicFloat(PARAM_REFINE_SEPARATION_WEIGHT);
	auto min_distance = params.getAbsPerc(PARAM_REFINE_SEPARATION_MIN_DISTANCE);

	vcg::tri::SelectionStack<CMeshO> selection(original);
	selection.push();

	//compute polylines
	std::vector< std::vector<vcg::Point3<Scalarm>> > polylines;
	{
		cb(0, "Computing polylines...");

		//find faces with unset facetag
		auto facetag = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(original, facetag_id.toStdString());
		std::vector<CFaceO*> untagged_faces;
		for (auto& face : original.face)
		{
			if(facetag[face] == -1)
				untagged_faces.emplace_back(&face);
		}

		//find untagged connected components
		std::vector< std::vector<CFaceO*> > connected_components;
		
		original.face.EnableMark();
		original.face.EnableFFAdjacency();
		vcg::tri::UpdateTopology<CMeshO>::FaceFace(original);
		vcg::tri::InitFaceIMark(original);
		vcg::tri::UnMarkAll(original);

		for (auto* face : untagged_faces)
		{
			if (vcg::tri::IsMarked(original, face))
				continue;

			connected_components.emplace_back();
			auto& component = connected_components.back();

			std::queue<CFaceO*> frontier;
			frontier.push(face);

			while ( !frontier.empty() )
			{
				auto* face = frontier.front(); frontier.pop();
				component.emplace_back(face);
				vcg::tri::Mark(original, face);

				std::vector<CFaceO*> star;
				vcg::face::FFExtendedStarFF(face, 1, star);

				for (auto* adj : star)
				{
					if (!vcg::tri::IsMarked(original, adj) && facetag[adj] == -1)
						frontier.push(adj);
				}
			}
		}

		original.face.DisableMark();
		original.face.DisableFFAdjacency();

		//foreach component
		vcg::tri::UpdateSelection<CMeshO>::Clear(original);
		for (auto& component : connected_components)
		{
			for (auto* face : component)
				face->SetS();

			CMeshO conn_comp;
			vcg::tri::UpdateSelection<CMeshO>::VertexFromFaceLoose(original);
			vcg::tri::Append<CMeshO, CMeshO>::MeshCopyConst(conn_comp, original, true);
			vcg::tri::UpdateSelection<CMeshO>::Clear(original);

			//extract polylines
			std::vector< std::vector<vcg::Point3<Scalarm>> > outlines;
			conn_comp.face.EnableFFAdjacency();
			vcg::tri::UpdateTopology<CMeshO>::FaceFace(conn_comp);
			vcg::tri::OutlineUtil<Scalarm>::ConvertMeshBoundaryToOutline3Vec(conn_comp, outlines);

			//remove longest polyline
			auto compute_polyline_lenght = [](std::vector<vcg::Point3<Scalarm>> const& poly)
			{
				Scalarm lenght = 0;
				for (int i = 0; i < poly.size() - 1; i++)
					lenght += vcg::Distance(poly[i], poly[i + 1]);
				lenght += vcg::Distance(poly[poly.size() - 1], poly[0]);
				return lenght;
			};

			std::sort(outlines.begin(), outlines.end(),
				[compute_polyline_lenght](std::vector<vcg::Point3<Scalarm>> const& l, std::vector<vcg::Point3<Scalarm>> const& r) {
					return compute_polyline_lenght(l) < compute_polyline_lenght(r);
				}
			);

			outlines.pop_back();
			std::copy(outlines.begin(), outlines.end(), std::back_inserter(polylines));
		}
	}

	//refine polylines
	std::vector< std::pair<PolylineMesh, vcg::Plane3<Scalarm>> > single_polylines(polylines.size());
	{
		cb(0, "Refining polylines...");

		PolylineMesh c_original;
		vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopyConst(c_original, original);
		auto com = vcg::tri::CoM<PolylineMesh>(c_original);
		com.Init();

		//convert polylines to PolylineMesh
		for (int i = 0; i < polylines.size(); i++)
		{
			auto& polyline_vec = polylines[i];
			auto& polyline = single_polylines[i].first;
			auto& fitting_plane = single_polylines[i].second;
			vcg::FitPlaneToPointSet(polyline_vec, fitting_plane);
			vcg::tri::OutlineUtil<Scalarm>::ConvertOutline3VecToEdgeMesh(polyline_vec, polyline);
		}

		//smooth-project
		for (int iter = 0; iter < num_iter; iter++)
		{
			cb( (iter + 1) * 100 / num_iter, "Refining polylines...");
			for (auto& pair : single_polylines)
			{
				auto& polyline = pair.first;
				auto& plane = pair.second;

				//smooth project polyline
				if (polyline.EN() > 0)
				{
					for (auto& pair2 : single_polylines) {
						auto& polyline2 = pair2.first;
						if (&polyline != &polyline2)
							movePolylinesApart(polyline, polyline2, min_distance, separation_w);
					}

					movePolylineToFittingPlane(polyline, plane, fit_plane_w);
					com.SmoothProject(polyline, 1, smooth_w, proj_w);
					vcg::tri::Clean<PolylineMesh>::RemoveUnreferencedVertex(polyline);
					vcg::tri::Allocator<PolylineMesh>::CompactEveryVector(polyline);
				}
			}
		}
	}

	auto* polylines_mm = document.addNewMesh(QString(), QString("Polylines"), false);
	for (auto& pair : single_polylines)
		vcg::tri::Append<CMeshO, PolylineMesh>::MeshAppendConst(polylines_mm->cm, pair.first);

	//cut branches based on polylines

	//close holes (for each polyline close at one side and copy the closing to the other branch)

	selection.pop();
	return {};
}

void movePolylinesApart(PolylineMesh& lpolyline, PolylineMesh const& rpolyline, Scalarm min_distance, Scalarm weight)
{
	for (auto& lvert : lpolyline.vert) {
		if (lvert.IsD()) continue;
		for (auto& redge : rpolyline.edge) {
			if (redge.IsD()) continue;

			Scalarm distance = min_distance;
			vcg::Point3<Scalarm> proj_point;
			if (vcg::edge::PointDistance(redge, lvert.cP(), distance, proj_point)) {
				auto difference_vec = lvert.cP() - proj_point;
				lvert.P() += difference_vec.normalized() * std::fmax(min_distance - distance, 0.0) * weight;
			}
		}
	}
}

void movePolylineToFittingPlane(PolylineMesh& polyline, vcg::Plane3<Scalarm> const& plane, Scalarm weight)
{
	for (auto& vert : polyline.vert)
	{
		auto proj_point = plane.Projection(vert.cP());
		auto difference_vec = proj_point - vert.cP();
		vert.P() += difference_vec * weight;
	}
}

}
