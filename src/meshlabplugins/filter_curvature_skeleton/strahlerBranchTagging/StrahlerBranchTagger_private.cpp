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

#include "StrahlerBranchTagger_private.h"

#include "SkeletonTreeBuilder.h"
#include "common/additionalAttributeNames.h"
#include <vector>
#include <vcg/complex/allocate.h>
#include <common/plugins/interfaces/filter_plugin.h>

typedef vcg::tri::Allocator<CMeshO> Allocator;

namespace curvatureSkeleton
{

StrahlerNumberCalculator::StrahlerNumberCalculator(CMeshO& tree_mesh)
    : tree(tree_mesh) { }

void StrahlerNumberCalculator::compute()
{
	auto tree_nodes    = SkeletonTreeBuilder::getTreeNodes(tree);
	auto tree_branches = SkeletonTreeBuilder::getTreeBranches(tree);

	auto node_numbers = Allocator::GetPerVertexAttribute<uint>(tree, ATTRIBUTE_TREE_NODE_STRAHLER_NUMBER);
	auto branch_numbers = Allocator::GetPerEdgeAttribute<uint>(tree, ATTRIBUTE_TREE_BRANCH_STRAHLER_NUMBER);

	std::vector<uint> current_nodes;
	for (int node_index = 0; node_index < tree.VN(); node_index++)
	{
		auto& node = tree_nodes[node_index];
		if ( node.next_branches.empty() )
			current_nodes.push_back(node_index);
	}

	std::vector<uint> temp;
	uint              current_number = 1;
	while ( !current_nodes.empty() )
	{		
		for (auto node_index : current_nodes)
		{
			auto& node = tree_nodes[node_index];
			node_numbers[node_index] = current_number;
			if (node.previous_branch.has_value())
			{
				auto& branch = tree_branches[ node.previous_branch.get() ];
				branch_numbers[ node.previous_branch.get() ] = current_number;
				temp.push_back(branch.previous_node);
			}
		}

		current_nodes.clear();
		current_nodes.swap(temp);
		current_number++;
	}
}

StrahlerBranchTagger::StrahlerNodeNumbers StrahlerNumberCalculator::getNodeNumbers(CMeshO const& tree_mesh)
{
	return Allocator::GetPerVertexAttribute<uint>(tree_mesh, ATTRIBUTE_TREE_NODE_STRAHLER_NUMBER);
}

StrahlerBranchTagger::StrahlerBranchNumbers StrahlerNumberCalculator::getBranchNumbers(CMeshO const& tree_mesh)
{
	return Allocator::GetPerEdgeAttribute<uint>(tree_mesh, ATTRIBUTE_TREE_BRANCH_STRAHLER_NUMBER);
}

StrahlerBranchTagger::StrahlerVertexNumbers StrahlerBranchTagger::getStrahlerNumbers(CMeshO const& mesh)
{
	return Allocator::GetPerVertexAttribute<uint>(mesh, ATTRIBUTE_VERTEX_STRAHLER_NUMBER);
}


SaveStrahlerNumber::SaveStrahlerNumber(CMeshO& original_mesh, CMeshO& skeleton, CMeshO const& tree)
	: original(original_mesh), skeleton(skeleton), tree(tree) { }

void SaveStrahlerNumber::saveNumbers()
{
	auto branches = SkeletonTreeBuilder::getTreeBranches(tree);
	auto numbers  = StrahlerNumberCalculator::getBranchNumbers(tree);
	auto skeleton_numbers  = Allocator::GetPerVertexAttribute<uint>(skeleton, ATTRIBUTE_VERTEX_STRAHLER_NUMBER);

	for (int i = 0; i < tree.EN(); i++)
	{
		auto current_number = numbers[i];
		for (auto index : branches[i].skeleton_vertices)
		{
			skeleton_numbers[index] = current_number;
		}
	}
	
	auto original_numbers     = Allocator::GetPerVertexAttribute<uint>(original, ATTRIBUTE_VERTEX_STRAHLER_NUMBER);
	auto original_to_skeleton = Allocator::GetPerVertexAttribute<uint>(original, ATTRIBUTE_MESH_TO_SKELETON_INDEX_NAME);
	for (int i = 0; i < original.VN(); i++)
	{
		original_numbers[i] = skeleton_numbers[ original_to_skeleton[i] ];
	}
}

void SaveStrahlerNumber::strahlerNumberToQuality(CMeshO& mesh)
{
	auto numbers = Allocator::FindPerVertexAttribute<uint>(mesh, ATTRIBUTE_VERTEX_STRAHLER_NUMBER);

	if (Allocator::IsValidHandle(mesh, numbers))
	{
		for (int i = 0; i < mesh.VN(); i++)
		{
			auto& vert = mesh.vert[i];
			vert.Q()   = numbers[i];
		}
	}	
	else
	{
		throw MLException("The selected mesh has no attribute by name \"" ATTRIBUTE_VERTEX_STRAHLER_NUMBER "\".");
	}
}

}
