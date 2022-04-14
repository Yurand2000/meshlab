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

#include "SkeletonTreeBuilder.h"
#include "SkeletonTreeBuilder_private.h"

namespace curvatureSkeleton
{

void SkeletonTreeBuilder::checkSkeletonTree(CMeshO const& skeleton)
{
	auto neighbors = TreeConnectivity().findConnectedVertices(skeleton);
	if (neighbors.size() != skeleton.vert.size())
		throw MLException("Cannot generate tree structure: could not determine the correct vertex graph. Has skeleton mesh been generated through the Skeletonization filter?");

	size_t visitedNodes = 0;
	bool isTree       = IsGraphATree(neighbors).isGraphATree(0, visitedNodes);
	bool isConnected  = visitedNodes == skeleton.vert.size();
	if (!isTree)
		throw MLException("Cannot generate tree structure: given skeleton has loops.");
	if (!isConnected)
		throw MLException("Cannot generate tree structure: given skeleton graph is not connected.");
}

void SkeletonTreeBuilder::generateTree(CMeshO& tree, CMeshO const& skeleton, uint root_index)
{
	auto neighbors = TreeConnectivity().findConnectedVertices(skeleton);
	if (neighbors.size() != skeleton.vert.size())
		throw MLException(
			"Cannot generate tree structure: could not determine the correct vertex graph. "
			"Has the skeleton mesh been generated through the Skeletonization filter?");
	TreeBuilder(tree, skeleton, neighbors).generateTree(root_index);
}

SkeletonTreeBuilder::SkeletonTreeNodes SkeletonTreeBuilder::getTreeNodes(CMeshO const& tree)
{
	return TreeBuilder::getTreeNodes(tree);
}

SkeletonTreeBuilder::SkeletonTreeBranches SkeletonTreeBuilder::getTreeBranches(CMeshO const& tree)
{
	return TreeBuilder::getTreeBranches(tree);
}

bool SkeletonTreeBuilder::SkeletonTreeNode::isRoot()
{
	return !previous_branch.has_value();
}

}
