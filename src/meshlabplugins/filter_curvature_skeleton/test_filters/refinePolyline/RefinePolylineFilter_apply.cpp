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

#include "RefinePolylineFilter.h"

#include "common/PolylineMesh.h"
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/curve_on_manifold.h>
#include <forward_list>

#define ATTRIBUTE_BRANCH_NUMBER "branch_number"

namespace curvatureSkeleton
{
static Scalarm extractAvgRadiusBranch(CMeshO& original, CMeshO& skel_branch);
static void movePolylineToParentBranch(PolylineMesh& polyline, CMeshO const& parent_branch, Scalarm avg_distance, Scalarm weight);
static void movePolylinesApart(PolylineMesh& lpolyline, PolylineMesh const& rpolyline, Scalarm max_distance, Scalarm weight);
static void polylineToParentBranchNormals(PolylineMesh& polyline, CMeshO const& parent_branch);
static CMeshO extractSkeletonBranch(CMeshO& skeleton, CMeshO& branch, int branch_num);

struct SinglePolylineData {
	PolylineMesh polyline;
	CMeshO parent_branch;
	Scalarm parent_branch_number;
	Scalarm avg_radius;

	SinglePolylineData() : polyline(), parent_branch() { }
};

std::map<std::string, QVariant> RefinePolylineTestFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& rich_params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos*		 cb)
{
	auto& original = document.getMesh( rich_params.getMeshId(PARAM_ORIGINAL_MESH) )->cm;
	auto& skeleton = document.getMesh( rich_params.getMeshId(PARAM_SKELETON_MESH) )->cm;
	auto& polylines_mesh = document.getMesh( rich_params.getMeshId(PARAM_POLYLINE_MESH) )->cm;
	auto num_iter = rich_params.getInt(PARAM_ITERATIONS); if (num_iter < 1) { num_iter = 1; }
	auto smooth_w = rich_params.getDynamicFloat(PARAM_SMOOTH_WEIGTH);
	auto proj_w = rich_params.getDynamicFloat(PARAM_PROJECT_WEIGTH);
	auto force_w = rich_params.getDynamicFloat(PARAM_FORCE_WEIGTH);
	auto separation_w = rich_params.getDynamicFloat(PARAM_SEPARATION_WEIGHT); 
	auto max_distance = original.trBB().Diag() * rich_params.getDynamicFloat(PARAM_SEPARATION_MAX_DISTANCE);

	auto attrib = vcg::tri::Allocator<CMeshO>::FindPerVertexAttribute<Scalarm>(polylines_mesh, ATTRIBUTE_BRANCH_NUMBER);
	if ( !vcg::tri::Allocator<CMeshO>::IsValidHandle(polylines_mesh, attrib) ) throw MLException("Attribute " ATTRIBUTE_BRANCH_NUMBER " not found for Polyline Mesh!");

	//convert meshes to PolylineMesh
	PolylineMesh converted_mesh, converted_polylines;
	vcg::tri::Allocator<PolylineMesh>::AddPerVertexAttribute<Scalarm>(converted_polylines, ATTRIBUTE_BRANCH_NUMBER);
	vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopy(converted_mesh, original);
	vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopy(converted_polylines, polylines_mesh);

	//prepare curve on manifold class
	auto com = vcg::tri::CoM<PolylineMesh>(converted_mesh);
	com.Init();

	//split polyline in connected components
	std::vector<std::pair<int, PolylineMesh::EdgeType*>> polylines_data;
	vcg::tri::Clean<PolylineMesh>::edgeMeshConnectedComponents(converted_polylines, polylines_data);

	PolylineMesh smoothed_polylines;
	vcg::tri::Allocator<PolylineMesh>::AddPerVertexAttribute<Scalarm>(smoothed_polylines, ATTRIBUTE_BRANCH_NUMBER);

	std::forward_list< SinglePolylineData > single_polylines;

	//split polyline mesh
	auto parent_branch_numbers = vcg::tri::Allocator<PolylineMesh>::GetPerVertexAttribute<Scalarm>(converted_polylines, ATTRIBUTE_BRANCH_NUMBER);
	for (size_t i = 0; i < polylines_data.size(); i++)
	{
		auto& polyline_data = polylines_data[i];
		single_polylines.emplace_front();
		auto& polyline = single_polylines.front().polyline;
		auto& parent_branch = single_polylines.front().parent_branch;
		auto& parent_branch_number = single_polylines.front().parent_branch_number;
		auto& avg_radius = single_polylines.front().avg_radius;

		//extract polyline
		auto it = vcg::tri::EdgeConnectedComponentIterator<PolylineMesh>();
		vcg::tri::UpdateSelection<PolylineMesh>::EdgeClear(converted_polylines);
		for (it.start(converted_polylines, polyline_data.second); !it.completed(); ++it) {
			(*it)->SetS();
		}
		vcg::tri::Allocator<PolylineMesh>::AddPerVertexAttribute<Scalarm>(polyline, ATTRIBUTE_BRANCH_NUMBER);
		vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshCopy(polyline, converted_polylines, true);
		vcg::tri::UpdateSelection<PolylineMesh>::Clear(polyline);

		//extract parent branch
		it.start(converted_polylines, polyline_data.second);
		parent_branch_number = parent_branch_numbers[(*it)->V(0)];
		extractSkeletonBranch(skeleton, parent_branch, parent_branch_number);
		avg_radius = extractAvgRadiusBranch(original, parent_branch);
	}

	//smooth-project
	for (int iter = 0; iter < num_iter; iter++)
	{
		for (auto it = single_polylines.begin(); it != single_polylines.end(); it++)
		{
			auto& polyline = (*it).polyline;
			auto& parent_branch = (*it).parent_branch;
			auto& avg_radius = (*it).avg_radius;

			//smooth project polyline
			for (auto it2 = single_polylines.begin(); it2 != single_polylines.end(); it2++) {
				if (it != it2)
					movePolylinesApart(polyline, (*it2).polyline, max_distance, separation_w);
			}
			movePolylineToParentBranch(polyline, parent_branch, avg_radius, force_w);
			com.SmoothProject(polyline, 1, smooth_w, proj_w);
			vcg::tri::Clean<PolylineMesh>::RemoveUnreferencedVertex(polyline);
			vcg::tri::Allocator<PolylineMesh>::CompactEveryVector(polyline);
		}
	}

	//cleanup
	for (auto& polyline_tuple : single_polylines)
	{
		auto& polyline = polyline_tuple.polyline;
		auto& parent_branch = polyline_tuple.parent_branch;
		auto& parent_branch_number = polyline_tuple.parent_branch_number;
		auto& avg_radius = polyline_tuple.avg_radius;

		//reassign correct parent index because new vertices may be added
		auto parent_branch_numbers = vcg::tri::Allocator<PolylineMesh>::GetPerVertexAttribute<Scalarm>(polyline, ATTRIBUTE_BRANCH_NUMBER);
		for (auto& vertex : polyline.vert)
			parent_branch_numbers[vertex] = parent_branch_number;

		//debug show normals
		polylineToParentBranchNormals(polyline, parent_branch);

		//append polyline
		vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshAppendConst(smoothed_polylines, polyline);
	}

	vcg::tri::Append<CMeshO, PolylineMesh>::MeshCopy(polylines_mesh, smoothed_polylines);
	vcg::tri::UpdateSelection<CMeshO>::Clear(polylines_mesh);

	return {};
}

Scalarm extractAvgRadiusBranch(CMeshO& original, CMeshO& skel_branch)
{
	Scalarm branch = 0;
	int count = 0;

	for (auto& skel_vertex : skel_branch.vert)
	{
		auto min_distance = std::numeric_limits<Scalarm>::max();
		auto position = vcg::Point3<Scalarm>();
		for (auto& vertex : original.vert)
		{
			auto distance = vcg::SquaredDistance(vertex.cP(), skel_vertex.cP());
			if (distance < min_distance) {
				min_distance = distance;
				position = skel_vertex.cP();
			}
		}

		branch += std::sqrt(min_distance);
		count += 1;
	}

	return (count > 0) ? branch / count : 0;
}

void movePolylinesApart(PolylineMesh& lpolyline, PolylineMesh const& rpolyline, Scalarm max_distance, Scalarm weight)
{
	for (auto& lvert : lpolyline.vert) {
		if (lvert.IsD()) continue;
		for (auto& redge : rpolyline.edge) {
			if (redge.IsD()) continue;

			Scalarm distance = max_distance;
			vcg::Point3<Scalarm> proj_point;
			if (vcg::edge::PointDistance(redge, lvert.cP(), distance, proj_point)) {
				auto difference_vec = proj_point - lvert.cP();
				lvert.P() += difference_vec * max_distance * 100.0 * weight;
			}
		}
	}
}

void movePolylineToParentBranch(PolylineMesh& polyline, CMeshO const& parent_branch, Scalarm avg_distance, Scalarm weight)
{
	if ( !parent_branch.vert.empty() )
	{
		for (auto& vertex : polyline.vert)
		{
			// compute closest point on parent branch
			auto min_point = 0;
			auto min_distance = std::numeric_limits<Scalarm>::max();
			for (auto& parent_vertex : parent_branch.vert)
			{
				auto distance = vcg::SquaredDistance(vertex.cP(), parent_vertex.cP());
				if (distance < min_distance) {
					min_distance = distance;
					min_point = parent_vertex.Index();
				}
			}

			// compute force delta
			auto difference_vec = parent_branch.vert[min_point].cP() - vertex.cP();
			auto normalized_vec = vcg::Normalized(difference_vec);
			auto delta_vec = difference_vec - normalized_vec * avg_distance;

			// move polyline vertex
			vertex.P() += delta_vec * weight / 100.0;
			vertex.N() = normalized_vec; //debug
		}
	}	
}

CMeshO extractSkeletonBranch(CMeshO& skeleton, CMeshO& branch, int branch_num)
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

	vcg::tri::Append<CMeshO, CMeshO>::MeshCopyConst(branch, skeleton, true);

	vcg::tri::UpdateSelection<CMeshO>::Clear(skeleton);
	vcg::tri::UpdateSelection<CMeshO>::Clear(branch);
	return branch;
}

void polylineToParentBranchNormals(PolylineMesh& polyline, CMeshO const& parent_branch)
{
	if (!parent_branch.vert.empty())
	{
		for (auto& vertex : polyline.vert)
		{
			// compute closest point on parent branch
			auto min_point = 0;
			auto min_distance = std::numeric_limits<Scalarm>::max();
			for (auto& parent_vertex : parent_branch.vert)
			{
				auto distance = vcg::SquaredDistance(vertex.cP(), parent_vertex.cP());
				if (distance < min_distance) {
					min_distance = distance;
					min_point = parent_vertex.Index();
				}
			}

			// compute force delta
			auto normalized_vec = vcg::Normalized(parent_branch.vert[min_point].cP() - vertex.cP());
			vertex.N() = normalized_vec;
		}
	}
}

}
