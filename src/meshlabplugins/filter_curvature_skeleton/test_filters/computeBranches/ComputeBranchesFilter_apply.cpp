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

#include "ComputeBranchesFilter.h"

#include "common/SkeletonMesh.h"
#include "common/ComputeBranches.h"

#define ATTRIBUTE_HACK_ORDER_NUMBER "hack_order_number"
#define ATTRIBUTE_ROOT_INDEX "root_index"
#define ATTRIBUTE_MESH_TO_SKELETON "skeleton_index"

namespace curvatureSkeleton
{

std::map<std::string, QVariant> ComputeBranchesTestFilter::applyFilter(
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

	//compute branches
	auto branches_data = ComputeBranches<CMeshO>::compute(original, skeleton, tree, tree_root_index, ATTRIBUTE_HACK_ORDER_NUMBER, std::greater<int>(), ATTRIBUTE_MESH_TO_SKELETON);

	auto branches = ComputeBranches<CMeshO>::extractBranches(original, branches_data);

	for (int i = 0; i < branches.size(); i++)
	{
		auto* new_branch = document.addNewMesh("", QString::asprintf("Branch %d", i), false);
		vcg::tri::Append<CMeshO, CMeshO>::Mesh(new_branch->cm, branches[i]);
		new_branch->setVisible(false);
	}

	return {};
}

}
