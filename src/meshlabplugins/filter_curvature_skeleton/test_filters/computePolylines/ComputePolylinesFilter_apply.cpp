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

#include "ComputePolylinesFilter.h"

#include "common/SkeletonMesh.h"
#include "common/BranchTagger.h"
#include "common/ComputeBranches.h"
#include "common/MeshBorderPolyline.h"

#define ATTRIBUTE_HACK_ORDER_NUMBER "hack_order_number"
#define ATTRIBUTE_ROOT_INDEX "root_index"
#define ATTRIBUTE_MESH_TO_SKELETON "skeleton_index"
#define ATTRIBUTE_BRANCH_NUMBER "branch_number"
#define ATTRIBUTE_BRANCH_ORDER "branch_order"

namespace curvatureSkeleton
{
static void copySkeletonBranch(
	std::vector<MeshBranch> const& branches_data, int branch,
	CMeshO& skeleton, CMeshO& skeleton_vertices
);

std::map<std::string, QVariant> ComputePolylinesTestFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& rich_params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos*		 cb)
{
	auto& original = document.getMesh(rich_params.getMeshId(PARAM_ORIGINAL_MESH))->cm;
	auto& skeleton = document.getMesh(rich_params.getMeshId(PARAM_SKELETON_MESH))->cm;
	auto& tree     = document.getMesh(rich_params.getMeshId(PARAM_TREE_MESH))->cm;
	auto  tree_root_index = vcg::tri::Allocator<CMeshO>::GetPerMeshAttribute<Scalarm>(tree, ATTRIBUTE_ROOT_INDEX)();

	//convert the tree to SkeletonMesh
	SkeletonMesh converted_tree;
	vcg::tri::Append<SkeletonMesh, CMeshO>::MeshCopyConst(converted_tree, tree);
	vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(converted_tree);
	vcg::tri::InitVertexIMark(converted_tree);
	vcg::tri::InitEdgeIMark(converted_tree);

	//compute order number
	vcg::tri::Stat<SkeletonMesh>::ComputeHackOrderNumbers(converted_tree, tree_root_index, ATTRIBUTE_HACK_ORDER_NUMBER);

	//update tree mesh
	vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<Scalarm>(tree, ATTRIBUTE_HACK_ORDER_NUMBER);
	vcg::tri::Append<CMeshO, SkeletonMesh>::MeshCopyConst(tree, converted_tree);
	vcg::tri::Allocator<CMeshO>::GetPerMeshAttribute<Scalarm>(tree, ATTRIBUTE_ROOT_INDEX)() = tree_root_index;

	//save attributes back to the original meshes
	BranchTagger<CMeshO>::copyAttributeTreeToSkeleton(skeleton, tree, tree_root_index, ATTRIBUTE_HACK_ORDER_NUMBER, true);
	BranchTagger<CMeshO>::copyAttributeUsingAdjacency(skeleton, original, ATTRIBUTE_HACK_ORDER_NUMBER, ATTRIBUTE_MESH_TO_SKELETON);

	//compute branches
	auto branches_data = ComputeBranches<CMeshO>::compute(original, skeleton, tree, tree_root_index, ATTRIBUTE_HACK_ORDER_NUMBER, std::greater<int>(), ATTRIBUTE_MESH_TO_SKELETON);
	auto branches = ComputeBranches<CMeshO>::extractBranches(original, branches_data);

	//prepare output meshes
	auto* polylines = document.addNewMesh("", QString::asprintf("Polylines"), false);
	vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<Scalarm>(polylines->cm, ATTRIBUTE_BRANCH_NUMBER);
	vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<Scalarm>(polylines->cm, ATTRIBUTE_BRANCH_ORDER);

	auto* skel_branches = document.addNewMesh("", QString::asprintf("Skeleton Branches"), false);
	vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<Scalarm>(skel_branches->cm, ATTRIBUTE_BRANCH_NUMBER);

	//build output meshes
	for (int i = 0; i < branches_data.size(); i++)
	{
		//build polyline parent branches
		copySkeletonBranch(branches_data, i, skeleton, skel_branches->cm);

		//build polylines
		if (branches_data[i].parent_branch_index < 0 ||
			branches_data[i].parent_branch_index >= branches_data.size()
		) continue;

		//compute polyline of branch
		branches[i].face.EnableFFAdjacency();
		auto polyline = MeshBorderPolyline<CMeshO>::getLongestPolyline(branches[i]);

		//associate parent branch index and branch order to each vertex of the polyline
		auto poly_number = vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<Scalarm>(polyline, ATTRIBUTE_BRANCH_NUMBER);
		auto poly_order_number = vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<Scalarm>(polyline, ATTRIBUTE_BRANCH_ORDER);
		for (auto& vertex : polyline.vert) {
			poly_number[vertex] = branches_data[i].parent_branch_index;
			poly_order_number[vertex] = branches_data[i].order_number;
		}

		//append polyline to polylines mesh
		vcg::tri::Append<CMeshO, CMeshO>::MeshAppendConst(polylines->cm, polyline);
	}

	return {};
}

void copySkeletonBranch(
	std::vector<MeshBranch> const& branches_data, int branch,
	CMeshO& skeleton, CMeshO& skeleton_vertices
) {
	auto skel_branch_number = vcg::tri::Allocator<CMeshO>::GetPerVertexAttribute<Scalarm>(skeleton, ATTRIBUTE_BRANCH_NUMBER);

	//associate branch index to each vertex of the skeleton
	for (auto vertex : branches_data[branch].skeleton_vertices) {
		skel_branch_number[vertex] = branch;
		skeleton.vert[vertex].SetS();
	}
	vcg::tri::Append<CMeshO, CMeshO>::MeshAppendConst(skeleton_vertices, skeleton, true);
	vcg::tri::UpdateSelection<CMeshO>::Clear(skeleton_vertices);
	vcg::tri::UpdateSelection<CMeshO>::Clear(skeleton);
}

}
