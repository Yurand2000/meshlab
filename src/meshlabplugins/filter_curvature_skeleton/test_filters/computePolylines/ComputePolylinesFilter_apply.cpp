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

namespace curvatureSkeleton
{

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

	auto* polylines = document.addNewMesh("", QString::asprintf("Polylines"), false);
	vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<Scalarm>(polylines->cm, ATTRIBUTE_BRANCH_NUMBER);
	auto skel_branch_number = vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<Scalarm>(skeleton, ATTRIBUTE_BRANCH_NUMBER);
	auto orig_branch_number = vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<Scalarm>(original, ATTRIBUTE_BRANCH_NUMBER);

	auto* skel_branches = document.addNewMesh("", QString::asprintf("Skeleton Branches"), false);
	auto skel_branches_number = vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<Scalarm>(skel_branches->cm, ATTRIBUTE_BRANCH_NUMBER);
	for (int i = 0; i < branches.size(); i++)
	{
		auto& branch = branches[i];

		//associate branch index to each vertex of the original mesh
		for (auto vertex : branches_data[i].mesh_vertices) {
			orig_branch_number[vertex] = i;
		}

		//associate branch index to each vertex of the skeleton
		for (auto vertex : branches_data[i].skeleton_vertices) {
			skel_branch_number[vertex] = i;
			skeleton.vert[vertex].SetS();
		}
		vcg::tri::Append<CMeshO, CMeshO>::MeshAppendConst(skel_branches->cm, skeleton, true);
		vcg::tri::UpdateSelection<CMeshO>::Clear(skeleton);

		//compute polyline of branch
		branch.face.EnableFFAdjacency();
		auto polyline = MeshBorderPolyline<CMeshO>::getLongestPolyline(branch);
		branch.face.DisableFFAdjacency();

		//associate parent branch index to each vertex of the polyline
		auto poly_number = vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<Scalarm>(polyline, ATTRIBUTE_BRANCH_NUMBER);
		for (auto& vertex : polyline.vert) {
			poly_number[vertex] = branches_data[i].parent_branch_index;
		}

		//append polyline to polylines mesh
		vcg::tri::Append<CMeshO, CMeshO>::MeshAppendConst(polylines->cm, polyline);
	}

	return {};
}

}
