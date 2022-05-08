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

#include "StrahlerBranchTagger.h"
#include "StrahlerBranchTagger_private.h"

#include <queue>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <vcg/complex/complex.h>

namespace curvatureSkeleton
{
typedef vcg::tri::Allocator<SkeletonMesh> SkeletonMeshAllocator;

void calculateStrahlerNumbers(SkeletonMesh& tree, int root_index)
{
	auto attribute = SkeletonMeshAllocator::GetPerVertexAttribute<uint>(tree, ATTRIBUTE_STRAHLER_NUMBER);

	struct StrahlerNode
	{
		int node;
		int parent;
	};
	std::stack<StrahlerNode> tree_reverse;
	std::unordered_map<SkeletonVertex*, int> skelton_to_index;
	for (int i = 0; i < tree.vert.size(); i++)
	{
		auto* vertex = &tree.vert[i];
		skelton_to_index.emplace(vertex, i);
		attribute[i] = 1;
	}

	std::queue<int> frontier;
	std::unordered_set<int> visited;
	frontier.push(root_index);
	do
	{
		auto index = frontier.front();
		visited.insert(index);
		frontier.pop();

		auto* vertex = &tree.vert[index];
		std::vector<SkeletonVertex*> verts;
		vcg::edge::VVStarVE(vertex, verts);

		for (auto* vert : verts)
		{
			auto v_index = skelton_to_index[vert];
			if (visited.count(v_index) == 0)
			{
				tree_reverse.push({v_index, index});
				frontier.push(v_index);
			}
		}
	}
	while ( !frontier.empty() );

	do
	{
		auto top = tree_reverse.top();
		tree_reverse.pop();

		auto curr_num = attribute[top.node];
		auto& parent_num = attribute[top.parent];
		if (parent_num < curr_num + 1)
			parent_num = curr_num + 1;
	}
	while ( !tree_reverse.empty() );
}
    
void StrahlerBranchTagger::calculateStrahlerNumbers(CMeshO& original, CMeshO& skeleton, CMeshO& tree)
{
	StrahlerNumberCalculator(tree).compute();
	SaveStrahlerNumber(original, skeleton, tree).saveNumbers();
}

StrahlerBranchTagger::StrahlerNodeNumbers StrahlerBranchTagger::getNodeNumbers(CMeshO const& tree_mesh)
{
	return StrahlerNumberCalculator::getNodeNumbers(tree_mesh);
}

StrahlerBranchTagger::StrahlerBranchNumbers StrahlerBranchTagger::getBranchNumbers(CMeshO const& tree_mesh)
{
	return StrahlerNumberCalculator::getBranchNumbers(tree_mesh);
}

void StrahlerBranchTagger::strahlerNumberToQuality(CMeshO& mesh)
{
	SaveStrahlerNumber::strahlerNumberToQuality(mesh);
}

}
