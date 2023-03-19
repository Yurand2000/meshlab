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
#include <vcg/complex/algorithms/crease_cut.h>
#include <vcg/complex/algorithms/hole.h>
#include <vcg/complex/algorithms/isotropic_remeshing.h>
#include <vcg/space/fitting3.h>

namespace curvatureSkeleton
{

static void movePolylineToFittingPlane(PolylineMesh& polyline, vcg::Plane3<Scalarm> const& plane, Scalarm weight);
static void movePolylinesApart(PolylineMesh& lpolyline, PolylineMesh const& rpolyline, Scalarm min_distance, Scalarm weight);
static Scalarm getMinVVDistance(PolylineMesh const& mesh0, PolylineMesh const& mesh1);
static void closeHoles(PolylineMesh& mesh, Scalarm refine_lenght);

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
	auto generate_polylines = params.getBool(PARAM_GENERATE_POLYLINES);
	auto refine_polylines = params.getBool(PARAM_DO_REFINE_POLYLINES);
	auto close_holes = params.getBool(PARAM_CLOSE_HOLES);

	//refine parameters
	auto num_iter = params.getInt(PARAM_REFINE_ITERATIONS); if (num_iter < 1) { num_iter = 1; }
	auto smooth_w = params.getDynamicFloat(PARAM_REFINE_SMOOTH_WEIGTH);
	auto proj_w = params.getDynamicFloat(PARAM_REFINE_PROJECT_WEIGTH);
	auto fit_plane_w = params.getDynamicFloat(PARAM_REFINE_FIT_PLANE_WEIGTH);
	auto separation_w = params.getDynamicFloat(PARAM_REFINE_SEPARATION_WEIGHT);
	auto min_distance = params.getAbsPerc(PARAM_REFINE_SEPARATION_MIN_DISTANCE);
	auto refine_hole_lenght = params.getAbsPerc(PARAM_REFINE_HOLE_EDGE_LENGHT);

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

	//convert polylines to PolylineMesh
	std::vector< PolylineMesh > polyline_meshes(polylines.size());
	for (int i = 0; i < polylines.size(); i++)
	{
		auto& polyline_vec = polylines[i];
		auto& polyline = polyline_meshes[i];
		vcg::tri::OutlineUtil<Scalarm>::ConvertOutline3VecToEdgeMesh(polyline_vec, polyline);
	}

	//refine polylines
	if(refine_polylines)
	{
		cb(0, "Refining polylines...");
		std::vector< std::pair<PolylineMesh, vcg::Plane3<Scalarm>> > single_polylines(polylines.size());

		for (int i = 0; i < polyline_meshes.size(); i++)
			std::swap(single_polylines[i].first, polyline_meshes[i]);

		PolylineMesh c_original;
		vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopyConst(c_original, original);
		auto com = vcg::tri::CoM<PolylineMesh>(c_original);
		com.Init();

		//generate fitting planes
		for (int i = 0; i < polylines.size(); i++)
		{
			auto& polyline_vec = polylines[i];
			auto& fitting_plane = single_polylines[i].second;
			vcg::FitPlaneToPointSet(polyline_vec, fitting_plane);
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

		for (int i = 0; i < single_polylines.size(); i++)
			std::swap(polyline_meshes[i], single_polylines[i].first);
	}

	//generate polylines mesh
	if (generate_polylines)
	{
		auto* polylines_mm = document.addNewMesh(QString(), QString("Polylines"), false);
		for (auto& polyline : polyline_meshes)
			vcg::tri::Append<CMeshO, PolylineMesh>::MeshAppendConst(polylines_mm->cm, polyline);
	}

	//cut branches based on polylines
	std::vector<PolylineMesh> pieces;
	{
		int current_polyline = 0;
		int num_polylines = polyline_meshes.size();
		cb(0, "Cutting polylines...");
		std::queue< std::pair<PolylineMesh, std::vector<PolylineMesh>> > split_queue;

		//first split-queue pair is the main mesh and all the (valid) polylines
		split_queue.emplace();
		PolylineMesh& c_original = split_queue.back().first;
		vcg::tri::Allocator<PolylineMesh>::AddPerFaceAttribute<Scalarm>(c_original, facetag_id.toStdString());
		vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopyConst(c_original, original);

		std::vector<PolylineMesh>& polylines = split_queue.back().second;
		polylines.reserve(polyline_meshes.size());
		for (auto& polyline : polyline_meshes)
		{
			if (polyline.EN() > 0)
			{
				polylines.emplace_back();
				std::swap(polylines.back(), polyline);
			}
		}

		//foreach mesh polylines pair
		while( !split_queue.empty() )
		{
			cb( (current_polyline + 1) * 100 / num_polylines, "Cutting polylines...");

			PolylineMesh mesh;
			std::vector<PolylineMesh> polylines;
			std::swap(split_queue.front().first, mesh);
			std::swap(split_queue.front().second, polylines);
			current_polyline++;
			split_queue.pop();

			//if no polylines to cut we are done
			if( polylines.empty() )
			{
				pieces.emplace_back();
				std::swap( pieces.back(), mesh );
			}
			else //cut on any polyline
			{
				auto& polyline = polylines[0];

				//prepare curve on manifold for the mesh
				vcg::tri::UpdateTopology<PolylineMesh>::FaceFace(mesh);
				vcg::tri::UpdateTopology<PolylineMesh>::VertexFace(mesh);
				vcg::tri::UpdateTopology<PolylineMesh>::VertexEdge(mesh);
				vcg::tri::UpdateSelection<PolylineMesh>::Clear(mesh);
				auto com = vcg::tri::CoM<PolylineMesh>(mesh);
				com.Init();

				//refine the mesh on the polyline and cut
				com.RefineCurveByBaseMesh(polyline);
				com.SplitMeshWithPolyline(polyline);
				com.TagFaceEdgeSelWithPolyLine(polyline);
				vcg::tri::CutMeshAlongSelectedFaceEdges(mesh);

				//get the two connected components of the cut mesh
				std::vector<std::pair<int, PolylineFace*>> conn_comps;
				vcg::tri::UpdateTopology<PolylineMesh>::FaceFace(mesh);
				vcg::tri::Clean<PolylineMesh>::ConnectedComponents(mesh, conn_comps);

				int index0 = 0, index1 = 1;
				if (conn_comps.size() > 2)
				{
					plugin.log("More than 2 connected components on single polyline!");
					break;
					//throw MLException("More than 2 connected components on single polyline!");
				}
				else if (conn_comps.size() < 2) {
					plugin.log("Less than 2 connected components on single polyline!");
					auto mesh = document.addNewMesh("", "Error Branch here", false);
					vcg::tri::Allocator<CMeshO>::AddVertex(mesh->cm, polyline.vert[0].cP());
					break;
					//throw MLException("Less than 2 connected components on single polyline!");
				}

				PolylineMesh part0, part1;

				//keep face tags on each piece
				vcg::tri::Allocator<PolylineMesh>::AddPerFaceAttribute<Scalarm>(part0, facetag_id.toStdString());
				vcg::tri::Allocator<PolylineMesh>::AddPerFaceAttribute<Scalarm>(part1, facetag_id.toStdString());

				conn_comps[index0].second->SetS();
				vcg::tri::UpdateSelection<PolylineMesh>::FaceConnectedFF(mesh);
				vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshCopy(part0, mesh, true);
				vcg::tri::UpdateSelection<PolylineMesh>::Clear(part0);
				vcg::tri::UpdateSelection<PolylineMesh>::Clear(mesh);

				conn_comps[index1].second->SetS();
				vcg::tri::UpdateSelection<PolylineMesh>::FaceConnectedFF(mesh);
				vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshCopy(part1, mesh, true);
				vcg::tri::UpdateSelection<PolylineMesh>::Clear(part1);
				vcg::tri::UpdateSelection<PolylineMesh>::Clear(mesh);

				//compute which of the remaining polylines cut the part 0 and which the part 1
				std::vector<PolylineMesh> part0_polys, part1_polys;

				//skip the first polyline because we used it to cut the mesh
				for (int i = 1; i < polylines.size(); i++)
				{
					auto& polyline = polylines[i];
					auto distance0 = getMinVVDistance(polyline, part0);
					auto distance1 = getMinVVDistance(polyline, part1);
					if (distance0 < distance1)
					{
						part0_polys.emplace_back();
						std::swap(polyline, part0_polys.back());
					}
					else
					{
						part1_polys.emplace_back();
						std::swap(polyline, part1_polys.back());
					}
				}

				if (closeHoles)
				{
					//close hole on the first piece
					closeHoles(part0, refine_hole_lenght);

					//duplicate the cap onto the other mesh and merge close vertices
					vcg::tri::UpdateSelection<PolylineMesh>::VertexFromFaceLoose(part0);
					vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshAppendConst(part1, part0, true);
					vcg::tri::Clean<PolylineMesh>::FlipMesh(part1, true);
					vcg::tri::Clean<PolylineMesh>::MergeCloseVertex(part1, 1.0E-7);
				}

				//add the new pieces to the queue
				vcg::tri::UpdateSelection<PolylineMesh>::Clear(part0);
				vcg::tri::UpdateSelection<PolylineMesh>::Clear(part1);

				split_queue.emplace();
				std::swap(split_queue.back().first, part0);
				std::swap(split_queue.back().second, part0_polys);

				split_queue.emplace();
				std::swap(split_queue.back().first, part1);
				std::swap(split_queue.back().second, part1_polys);
			}
		}
	}

	//for each new piece add a new mesh
	for (int i = 0; i < pieces.size(); i++)
	{
		auto& piece = pieces[i];
		auto* piece_mm = document.addNewMesh(QString(), QString("Piece %1").arg(i), false);
		vcg::tri::Allocator<CMeshO>::AddPerFaceAttribute<Scalarm>(piece_mm->cm, facetag_id.toStdString());
		vcg::tri::Append<CMeshO, PolylineMesh>::MeshAppendConst(piece_mm->cm, piece);
		piece_mm->updateBoxAndNormals();
	}

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

Scalarm getMinVVDistance(PolylineMesh const& mesh0, PolylineMesh const& mesh1)
{
	Scalarm min_dist = std::numeric_limits<Scalarm>::max();

	for (auto& vert0 : mesh0.vert)
	{
		for (auto& vert1 : mesh1.vert)
		{
			auto distance = vcg::SquaredDistance(vert0.cP(), vert1.cP());
			if (distance < min_dist)
				min_dist = distance;
		}
	}

	min_dist = vcg::math::Sqrt(min_dist);
	return min_dist;
}

void closeHoles(PolylineMesh& mesh, Scalarm refine_lenght)
{
	//code adapted from meshfilter.cpp

	size_t original_faces_count = mesh.face.size();
	auto max_hole_size = 50000;

	int holes_closed = 0; int stack_size = 0;
	vcg::tri::SelectionStack<PolylineMesh> selection(mesh);
	vcg::tri::UpdateSelection<PolylineMesh>::Clear(mesh);
	do
	{
		//close holes
		vcg::tri::UpdateTopology<PolylineMesh>::FaceFace(mesh);
		holes_closed = vcg::tri::Hole<PolylineMesh>::EarCuttingFill< vcg::tri::MinimumWeightEar<PolylineMesh> >(mesh, max_hole_size, false, &vcg::DummyCallBackPos);

		//select newly added faces
		vcg::tri::UpdateSelection<PolylineMesh>::FaceClear(mesh);
		for (int i = original_faces_count; i < mesh.face.size(); i++)
		{
			auto& face = mesh.face[i];
			if (!face.IsD())
				face.SetS();
		}

		//save selection in stack to restore it later
		selection.push();
		stack_size++;
	} while (holes_closed != 0);

	//select all the newly added faces
	for (int i = 0; i < stack_size; i++)
		selection.popOr();

	//refine newly added faces
	vcg::tri::IsotropicRemeshing<PolylineMesh>::Params params;
	params.SetFeatureAngleDeg(181.0f);
	params.adapt = false;
	params.selectedOnly = true;
	params.splitFlag = true;
	params.collapseFlag = true;
	params.swapFlag = true;
	params.smoothFlag = true;
	params.projectFlag = false;
	params.surfDistCheck = false;

	for (int k = 0; k < 3; k++)
	{
		params.SetTargetLen(refine_lenght * 3.0); params.iter = 5;
		vcg::tri::IsotropicRemeshing<PolylineMesh>::Do(mesh, params);

		params.SetTargetLen(refine_lenght / 3.0); params.iter = 3;
		vcg::tri::IsotropicRemeshing<PolylineMesh>::Do(mesh, params);

		params.SetTargetLen(refine_lenght); params.iter = 2;
		vcg::tri::IsotropicRemeshing<PolylineMesh>::Do(mesh, params);
	}
}

}
