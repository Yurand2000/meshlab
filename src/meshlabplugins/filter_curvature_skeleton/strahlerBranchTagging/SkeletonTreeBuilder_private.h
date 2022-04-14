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

#ifndef FILTERCURVATURESKELETON_SKELETON_TREE_BUILDER_PRIVATE
#define FILTERCURVATURESKELETON_SKELETON_TREE_BUILDER_PRIVATE

#include "SkeletonTreeBuilder.h"

#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <common/plugins/interfaces/filter_plugin.h>

#define ATTRIBUTE_TREE_NODE_STRUCT "tree_node_struct"
#define ATTRIBUTE_TREE_BRANCH_STRUCT "tree_branch_struct"

namespace curvatureSkeleton
{

class TreeConnectivity
{
public:
	typedef std::unordered_map<CVertexO const*, uint>          NodeIndices;
	typedef std::unordered_map<uint, std::unordered_set<uint>> NodeNeighbors;

public:
	NodeIndices   findVertexIndices(CMeshO const& skeleton);
	NodeNeighbors findConnectedVertices(CMeshO const& skeleton, NodeIndices const& indices);
	NodeNeighbors findConnectedVertices(CMeshO const& skeleton);
};

class IsGraphATree
{
public:
	typedef TreeConnectivity::NodeNeighbors NodeNeighbors;

private:
	struct GraphTreeData
	{
		uint parent_node;
		uint current_node;
	};
	typedef std::queue<GraphTreeData> FrontierNodes;
	typedef std::unordered_set<uint>  VisitedNodes;

public:
	IsGraphATree(NodeNeighbors const& neighbors);

	bool isGraphATree(uint start_node, size_t& return_visited_nodes);

private:
	NodeNeighbors const& neighbors;
};

class TreeBuilder
{
public:
	typedef TreeConnectivity::NodeNeighbors                         NodeNeighbors;
	typedef CMeshO::PerVertexAttributeHandle<SkeletonTreeBuilder::SkeletonTreeNode> SkeletonTreeNodes;
	typedef CMeshO::PerEdgeAttributeHandle<SkeletonTreeBuilder::SkeletonTreeBranch> SkeletonTreeBranches;

	struct ExpandVertexData
	{
		uint              skeleton_parent_vertex;
		uint              skeleton_current_vertex;
		uint              previous_node;
		std::vector<uint> current_branch;
	};
	typedef std::queue<ExpandVertexData> ExpandVertexFrontier;

public:
	TreeBuilder(CMeshO& tree, CMeshO const& skeleton, NodeNeighbors const& neighbors);

	void generateTree(uint root_index);

	static SkeletonTreeBuilder::SkeletonTreeNodes    getTreeNodes(CMeshO const& tree);
	static SkeletonTreeBuilder::SkeletonTreeBranches getTreeBranches(CMeshO const& tree);

private:
	uint closeCurrentBranch(ExpandVertexData node);

private:
	CMeshO&              tree;
	SkeletonTreeNodes    nodes;
	SkeletonTreeBranches branches;
	CMeshO const&        skeleton;
	NodeNeighbors const& neighbors;
};

}

#endif //FILTERCURVATURESKELETON_SKELETON_TREE_BUILDER_PRIVATE
