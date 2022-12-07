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

#define ATTRIBUTE_BRANCH_NUMBER "branch_number"

namespace curvatureSkeleton
{
static std::vector<std::pair<vcg::Point3<Scalarm>, Scalarm>> extractMinRadiusBranch(CMeshO& original, CMeshO& skel_branch);
static void movePolylineToParentBranch(PolylineMesh& polyline, std::vector<std::pair<vcg::Point3<Scalarm>, Scalarm>>& min_radius, Scalarm weight);
static CMeshO extractSkeletonBranch(CMeshO& skeleton, int branch_num);

std::map<std::string, QVariant> RefinePolylineTestFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& rich_params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos*		 cb)
{
	auto& original = document.getMesh( rich_params.getMeshId(PARAM_ORIGINAL_MESH) )->cm;
	auto& skeleton = document.getMesh(rich_params.getMeshId(PARAM_SKELETON_MESH))->cm;
	auto& polylines_mesh = document.getMesh( rich_params.getMeshId(PARAM_POLYLINE_MESH) )->cm;
	auto iter = rich_params.getInt(PARAM_ITERATIONS); if (iter < 1) { iter = 1; }
	auto smooth_w = rich_params.getDynamicFloat(PARAM_SMOOTH_WEIGTH);
	auto proj_w = rich_params.getDynamicFloat(PARAM_PROJECT_WEIGTH);
	auto force_w = rich_params.getDynamicFloat(PARAM_FORCE_WEIGTH);

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

	PolylineMesh smoothed_polylines, polyline;
	auto parent_branch_num = vcg::tri::Allocator<PolylineMesh>::AddPerVertexAttribute<Scalarm>(polyline, ATTRIBUTE_BRANCH_NUMBER);
	vcg::tri::Allocator<PolylineMesh>::AddPerVertexAttribute<Scalarm>(smoothed_polylines, ATTRIBUTE_BRANCH_NUMBER);
	for (auto& polyline_data : polylines_data)
	{
		//extract polyline
		auto it = vcg::tri::EdgeConnectedComponentIterator<PolylineMesh>();
		vcg::tri::UpdateSelection<PolylineMesh>::EdgeClear(converted_polylines);
		for (it.start(converted_polylines, polyline_data.second); !it.completed(); ++it) {
			(*it)->SetS();
		}
		vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshCopy(polyline, converted_polylines, true);
		vcg::tri::UpdateSelection<PolylineMesh>::Clear(polyline);

		//extract parent branch
		auto parent = extractSkeletonBranch(skeleton, parent_branch_num[0]);
		auto min_radius = extractMinRadiusBranch(original, parent);

		//smooth project polyline
		for (int i = 0; i < iter; i++) {
			movePolylineToParentBranch(polyline, min_radius, force_w);
			com.SmoothProject(polyline, 1, smooth_w, proj_w);
			vcg::tri::Clean<PolylineMesh>::RemoveUnreferencedVertex(polyline);
			vcg::tri::Allocator<PolylineMesh>::CompactEveryVector(polyline);
		}

		//append polyline
		vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshAppendConst(smoothed_polylines, polyline);
	}

	vcg::tri::Append<CMeshO, PolylineMesh>::MeshCopy(polylines_mesh, smoothed_polylines);
	vcg::tri::UpdateSelection<CMeshO>::Clear(polylines_mesh);

	return {};
}

std::vector<std::pair<vcg::Point3<Scalarm>, Scalarm>> extractMinRadiusBranch(CMeshO& original, CMeshO& skel_branch)
{
	std::vector<std::pair<vcg::Point3<Scalarm>, Scalarm>> branch;

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

		branch.push_back( std::make_pair(position, std::sqrt(min_distance)) );
	}

	return branch;
}

void movePolylineToParentBranch(PolylineMesh& polyline, std::vector<std::pair<vcg::Point3<Scalarm>, Scalarm>>& parent_branch, Scalarm weight)
{
	if ( !parent_branch.empty() )
	{
		for (auto& vertex : polyline.vert)
		{
			// compute closest point on parent branch
			auto min_point = 0;
			auto min_distance = std::numeric_limits<Scalarm>::max();
			for (int i = 0; i < parent_branch.size(); i++)
			{
				auto& pair = parent_branch[i];
				auto distance = vcg::SquaredDistance(vertex.cP(), pair.first);
				if (distance < min_distance) {
					min_distance = distance;
					min_point = i;
				}
			}

			// compute force delta
			/*
			auto difference_vec = min_radius[min_point].first - vertex.cP();
			auto normalized_vec = vcg::Normalized(difference_vec);
			auto delta_vec = difference_vec - normalized_vec * min_radius[min_point].second;
			*/
			auto delta_vec = parent_branch[min_point].first - vertex.cP();

			// move polyline vertex
			vertex.P() += delta_vec * weight / 100.0;
		}
	}	
}

CMeshO extractSkeletonBranch(CMeshO& skeleton, int branch_num)
{
	CMeshO branch;

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

}
