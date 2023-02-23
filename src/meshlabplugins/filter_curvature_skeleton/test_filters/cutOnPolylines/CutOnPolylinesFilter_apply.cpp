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
#include <chrono>

#define ATTRIBUTE_BRANCH_NUMBER "branch_number"
#define ATTRIBUTE_BRANCH_ORDER "branch_order"

namespace curvatureSkeleton
{

static void extractSkeletonBranch(CMeshO& skeleton, CMeshO& out_branch, int branch_num);
template<typename LMESH, typename RMESH> Scalarm minimumDistance(LMESH const& vert_mesh, RMESH const& mesh);

static void logExecutionTime(FilterPlugin const& plugin, std::chrono::steady_clock::time_point& clock, const char* msg);

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
	vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopy(converted_mesh, original);
	vcg::tri::Allocator<PolylineMesh>::AddPerVertexAttribute<Scalarm>(converted_polylines, ATTRIBUTE_BRANCH_NUMBER);
	vcg::tri::Allocator<PolylineMesh>::AddPerVertexAttribute<Scalarm>(converted_polylines, ATTRIBUTE_BRANCH_ORDER);
	vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopy(converted_polylines, polylines_mesh);

	//split polyline in connected components
	std::vector<std::pair<int, PolylineMesh::EdgeType*>> polylines_data;
	vcg::tri::Clean<PolylineMesh>::edgeMeshConnectedComponents(converted_polylines, polylines_data);

	auto clock = std::chrono::steady_clock::now();

	std::vector<std::tuple<PolylineMesh, Scalarm, Scalarm>> polylines;
	polylines.resize(polylines_data.size());
	for (int i = 0; i < polylines_data.size(); i++)
	{
		auto& polyline_data = polylines_data[i];
		auto& polyline = std::get<0>(polylines[i]);
		auto& polyline_order = std::get<1>(polylines[i]);
		auto& polyline_parent_branch = std::get<2>(polylines[i]);

		auto vertex = polyline_data.second->V(0);
		polyline_order = vcg::tri::Allocator<PolylineMesh>::GetPerVertexAttribute<Scalarm>(converted_polylines, ATTRIBUTE_BRANCH_ORDER)[vertex];
		polyline_parent_branch = vcg::tri::Allocator<PolylineMesh>::GetPerVertexAttribute<Scalarm>(converted_polylines, ATTRIBUTE_BRANCH_NUMBER)[vertex];

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
	}

	logExecutionTime(plugin, clock, "Polyline Extraction");

	//sort polylines by their order number
	std::sort(polylines.begin(), polylines.end(), [](const std::tuple<PolylineMesh, Scalarm, Scalarm>& l, const std::tuple<PolylineMesh, Scalarm, Scalarm>& r) {
		auto const& l_order = std::get<1>(l);
		auto const& r_order = std::get<1>(r);
		return l_order > r_order;
	});

	logExecutionTime(plugin, clock, "Polyline Sorting");

	//prepare curve on manifold class
	vcg::tri::UpdateTopology<PolylineMesh>::FaceFace(converted_mesh);
	vcg::tri::UpdateTopology<PolylineMesh>::VertexFace(converted_mesh);
	vcg::tri::UpdateTopology<PolylineMesh>::VertexEdge(converted_mesh);
	auto com = vcg::tri::CoM<PolylineMesh>(converted_mesh);
	com.Init();

	//refine mesh by polylines
	for (auto& polyline_tuple : polylines)
	{
		auto& polyline = std::get<0>(polyline_tuple);
		com.RefineCurveByBaseMesh(polyline);
		com.SplitMeshWithPolyline(polyline);
	}

	logExecutionTime(plugin, clock, "Polyline Refining");

	//cut by polyline
	std::vector<std::tuple<PolylineMesh, Scalarm, Scalarm>> cut_branches;
	cut_branches.reserve( polylines.size() + 1 );
	for (auto& polyline_tuple : polylines)
	{
		auto& polyline = std::get<0>(polyline_tuple);
		auto& polyline_order = std::get<1>(polyline_tuple);
		auto& polyline_parent_branch = std::get<2>(polyline_tuple);

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
		else if (conn_comps.size() < 2) {
			plugin.log("Less than 2 connected components on single polyline!");
			plugin.log("Last polyline hack num: %.0f", vcg::tri::Allocator<PolylineMesh>::GetPerVertexAttribute<Scalarm>(polyline, ATTRIBUTE_BRANCH_ORDER)[0]);
			auto mesh = document.addNewMesh("", "Error Branch here", false);
			vcg::tri::Allocator<CMeshO>::AddVertex(mesh->cm, polyline.vert[0].cP());
			break;
			//throw MLException("Less than 2 connected components on single polyline!");
		}

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
		extractSkeletonBranch(skeleton, skeleton_parent, polyline_parent_branch);

		if (minimumDistance(skeleton_parent, mesh0) < minimumDistance(skeleton_parent, mesh1)) {
			cut_branches.emplace_back(std::move(mesh1), polyline_order, -1);
			converted_mesh = std::move(mesh0);
		}
		else {
			cut_branches.emplace_back(std::move(mesh0), polyline_order, -1);
			converted_mesh = std::move(mesh1);
		}
	}

	//the remaining piece is the main branch.
	cut_branches.emplace_back(std::move(converted_mesh), 1, -1);

	logExecutionTime(plugin, clock, "Polyline Cutting");

	//find parent branches
	for (int i = 0; i < (cut_branches.size() - 1); i++) {
		auto& parent_number = std::get<2>(cut_branches[i]);
		auto& order_number = std::get<1>(cut_branches[i]);
		auto& polyline = std::get<0>(polylines[i]);

		//the parent branch is the one with the minimum distance from the specific cut polyline of any given branch,
		//provided that its order number is the one of the current branch minus one
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

	logExecutionTime(plugin, clock, "Find Parent Branches");

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

	logExecutionTime(plugin, clock, "Colorization and Saving");

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

void logExecutionTime(FilterPlugin const& plugin, std::chrono::steady_clock::time_point& clock, const char* msg)
{
	auto now = std::chrono::steady_clock::now();
	plugin.log("%s, time: %.2f", msg, std::chrono::duration<float, std::milli>(now - clock).count());
	clock = now;
}

}
