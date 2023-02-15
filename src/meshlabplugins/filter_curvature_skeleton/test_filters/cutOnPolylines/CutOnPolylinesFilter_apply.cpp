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

#include "CutOnPolylinesFilter.h"

#include "common/PolylineMesh.h"
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/curve_on_manifold.h>
#include <vcg/complex/algorithms/crease_cut.h>
#include <vcg/space/color4.h>

#define ATTRIBUTE_BRANCH_NUMBER "branch_number"
#define ATTRIBUTE_BRANCH_ORDER "branch_order"

namespace curvatureSkeleton
{

static void extractSkeletonBranch(CMeshO& skeleton, CMeshO& out_branch, int branch_num);
template<typename LMESH, typename RMESH> Scalarm minimumDistance(LMESH const& vert_mesh, RMESH const& mesh);

std::map<std::string, QVariant> CutOnPolylinesTestFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& rich_params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos*		 cb)
{
	auto& original = document.getMesh( rich_params.getMeshId(PARAM_ORIGINAL_MESH) )->cm;
	auto& skeleton = document.getMesh(rich_params.getMeshId(PARAM_SKELETON_MESH))->cm;
	auto& polylines_mesh = document.getMesh( rich_params.getMeshId(PARAM_POLYLINE_MESH) )->cm;
	auto strict = rich_params.getBool(PARAM_STRICT);

	//convert meshes to PolylineMesh
	PolylineMesh converted_mesh, converted_polylines;
	vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopy(converted_mesh, original); //COPY ATTRIBUTES?
	vcg::tri::Allocator<PolylineMesh>::AddPerVertexAttribute<Scalarm>(converted_polylines, ATTRIBUTE_BRANCH_NUMBER);
	vcg::tri::Allocator<PolylineMesh>::AddPerVertexAttribute<Scalarm>(converted_polylines, ATTRIBUTE_BRANCH_ORDER);
	vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopy(converted_polylines, polylines_mesh);

	//prepare curve on manifold class
	vcg::tri::UpdateTopology<PolylineMesh>::FaceFace(converted_mesh);
	vcg::tri::UpdateTopology<PolylineMesh>::VertexFace(converted_mesh);
	vcg::tri::UpdateTopology<PolylineMesh>::VertexEdge(converted_mesh);
	auto com = vcg::tri::CoM<PolylineMesh>(converted_mesh);
	com.Init();

	//split polyline in connected components
	std::vector<std::pair<int, PolylineMesh::EdgeType*>> polylines_data;
	vcg::tri::Clean<PolylineMesh>::edgeMeshConnectedComponents(converted_polylines, polylines_data);
	auto parent_branch_numbers = vcg::tri::Allocator<PolylineMesh>::GetPerVertexAttribute<Scalarm>(converted_polylines, ATTRIBUTE_BRANCH_NUMBER);

	std::vector<PolylineMesh> polylines;
	polylines.resize(polylines_data.size());
	for (int i = 0; i < polylines_data.size(); i++)
	{
		auto& polyline_data = polylines_data[i];
		auto& polyline = polylines[i];

		vcg::tri::Allocator<PolylineMesh>::AddPerVertexAttribute<Scalarm>(polyline, ATTRIBUTE_BRANCH_NUMBER);
		vcg::tri::Allocator<PolylineMesh>::AddPerVertexAttribute<Scalarm>(polyline, ATTRIBUTE_BRANCH_ORDER);

		//extract polyline
		auto it = vcg::tri::EdgeConnectedComponentIterator<PolylineMesh>();
		vcg::tri::UpdateSelection<PolylineMesh>::EdgeClear(converted_polylines);
		vcg::tri::UpdateSelection<PolylineMesh>::VertexClear(converted_polylines);
		for (it.start(converted_polylines, polyline_data.second); !it.completed(); ++it) {
			(*it)->SetS();
		}
		vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshCopy(polyline, converted_polylines, true);
		vcg::tri::UpdateTopology<PolylineMesh>::VertexEdge(polyline);
		vcg::tri::UpdateSelection<PolylineMesh>::Clear(polyline);

		//refine mesh by polyline
		com.RefineCurveByBaseMesh(polyline);
		com.SplitMeshWithPolyline(polyline);
	}

	std::sort(polylines.begin(), polylines.end(), [](const PolylineMesh& l, const PolylineMesh& r) {
		auto l_number = vcg::tri::Allocator<PolylineMesh>::GetPerVertexAttribute<Scalarm>(l, ATTRIBUTE_BRANCH_ORDER)[0];
		auto r_number = vcg::tri::Allocator<PolylineMesh>::GetPerVertexAttribute<Scalarm>(r, ATTRIBUTE_BRANCH_ORDER)[0];
		return l_number > r_number;
	});

	std::vector<std::tuple<PolylineMesh, Scalarm, Scalarm>> cut_branches;
	cut_branches.reserve( polylines.size() + 1 );
	for (auto& polyline : polylines)
	{
		//initialize com
		auto com = vcg::tri::CoM<PolylineMesh>(converted_mesh);
		com.Init();

		//cut base mesh on polyline
		com.TagFaceEdgeSelWithPolyLine(polyline);
		CutMeshAlongSelectedFaceEdges(converted_mesh);

		//get the two connected components
		std::vector<std::pair<int, PolylineFace*>> conn_comps;
		vcg::tri::UpdateTopology<PolylineMesh>::FaceFace(converted_mesh);
		vcg::tri::Clean<PolylineMesh>::ConnectedComponents(converted_mesh, conn_comps);

		int index0 = 0, index1 = 1;
		if (conn_comps.size() > 2) {
			if (strict)
				throw MLException("More than 2 connected components on single polyline!");
			else //use the two biggest connected components
			{
				int count0 = 0, count1 = 0;
				for (int i = 0; i < conn_comps.size(); i++) {
					conn_comps[i].second->SetS();
					auto size = vcg::tri::UpdateSelection<PolylineMesh>::FaceConnectedFF(converted_mesh);
					vcg::tri::UpdateSelection<PolylineMesh>::FaceClear(converted_mesh);

					if (size > count0) {
						index1 = index0;
						count1 = count0;
						index0 = i;
						count0 = size;
					}
					else if(size > count1) {
						index1 = i;
						count1 = size;
					}
				}
			}
		}
		else if (conn_comps.size() < 2)
			throw MLException("Less than 2 connected components on single polyline!");

		PolylineMesh mesh0, mesh1;

		conn_comps[index0].second->SetS();
		vcg::tri::UpdateSelection<PolylineMesh>::FaceConnectedFF(converted_mesh);
		vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshCopy(mesh0, converted_mesh, true);
		vcg::tri::UpdateSelection<PolylineMesh>::Clear(mesh0);
		vcg::tri::UpdateSelection<PolylineMesh>::Clear(converted_mesh);

		conn_comps[index1].second->SetS();
		vcg::tri::UpdateSelection<PolylineMesh>::FaceConnectedFF(converted_mesh);
		vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshCopy(mesh1, converted_mesh, true);
		vcg::tri::UpdateSelection<PolylineMesh>::Clear(mesh1);
		vcg::tri::UpdateSelection<PolylineMesh>::Clear(converted_mesh);

		//find which connected component is the leaf
		CMeshO skeleton_parent;
		auto parent_branch_number = vcg::tri::Allocator<PolylineMesh>::GetPerVertexAttribute<Scalarm>(polyline, ATTRIBUTE_BRANCH_NUMBER)[0];
		extractSkeletonBranch(skeleton, skeleton_parent, parent_branch_number);

		if (minimumDistance(skeleton_parent, mesh0) < minimumDistance(skeleton_parent, mesh1)) {
			cut_branches.emplace_back(std::move(mesh1), 0, -1);
			converted_mesh = std::move(mesh0);
		}
		else {
			cut_branches.emplace_back(std::move(mesh0), 0, -1);
			converted_mesh = std::move(mesh1);
		}

		//save hack number for the cut branch
		std::get<1>(cut_branches.back()) =
			vcg::tri::Allocator<PolylineMesh>::GetPerVertexAttribute<Scalarm>(polyline, ATTRIBUTE_BRANCH_ORDER)[0];
	}

	//the remaining piece is the main branch.
	cut_branches.emplace_back(std::move(converted_mesh), 1, -1);

	//find parent branches
	for (int i = 0; i < polylines.size(); i++) {
		auto& parent_number = std::get<2>(cut_branches[i]);
		auto& order_number = std::get<1>(cut_branches[i]);
		auto& polyline = polylines[i];

		Scalarm min_distance = std::numeric_limits<Scalarm>::max();
		for (int j = 0; j < cut_branches.size(); j++) {
			if (i != j && (order_number - 1) == std::get<1>(cut_branches[j])) {
				auto distance = minimumDistance(polyline, std::get<0>(cut_branches[j]));
				if (distance < min_distance) {
					min_distance = distance;
					parent_number = j;
				}
			}
		}
	}

	//colorize by hack numbers
	auto num_colors = std::get<1>(cut_branches[0]);
	for (int i = 0; i < cut_branches.size(); i++)
	{
		auto color = vcg::Color4b::Scatter(num_colors, (std::get<1>(cut_branches[i]) - 1));
		for ( auto& vert : std::get<0>(cut_branches[i]).vert ) {
			vert.C() = color;
		}
	}

	//save connected components
	for (int i = 0; i < cut_branches.size(); i++)
	{
		auto& branch = std::get<0>(cut_branches[i]);
		auto hack_number = std::get<1>(cut_branches[i]);
		auto parent_branch = std::get<2>(cut_branches[i]);
		auto new_mesh = document.addNewMesh("", QString::asprintf("Branch %d; Hack %.0f; Parent: %.0f", i, hack_number, parent_branch), false);
		new_mesh->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
		vcg::tri::Append<CMeshO, PolylineMesh>::MeshCopy(new_mesh->cm, branch);
	}

	return {};
}

void extractSkeletonBranch(CMeshO& skeleton, CMeshO& out_branch, int branch_num)
{
	auto branch_number = vcg::tri::Allocator<CMeshO>::FindPerVertexAttribute<Scalarm>(skeleton, ATTRIBUTE_BRANCH_NUMBER);
	if (!vcg::tri::Allocator<CMeshO>::IsValidHandle(skeleton, branch_number)) {
		throw new MLException("ATTRIBUTE MISSING ERROR!");
	}

	for (auto& vertex : skeleton.vert) {
		if (branch_number[vertex] == branch_num) {
			vertex.SetS();
		}
	}

	vcg::tri::Append<CMeshO, CMeshO>::MeshCopyConst(out_branch, skeleton, true);

	vcg::tri::UpdateSelection<CMeshO>::Clear(skeleton);
	vcg::tri::UpdateSelection<CMeshO>::Clear(out_branch);
}

template<typename LMESH, typename RMESH> Scalarm minimumDistance(LMESH const& vert_mesh, RMESH const& mesh)
{
	Scalarm min_distance = std::numeric_limits<Scalarm>::max();
	for (auto& vert : vert_mesh.vert) {
		for (auto& face : mesh.face) {
			vcg::Point3<Scalarm> closest_point;
			vcg::face::PointDistanceBase(face, vert.cP(), min_distance, closest_point);
		}
	}

	return min_distance;
}

}
