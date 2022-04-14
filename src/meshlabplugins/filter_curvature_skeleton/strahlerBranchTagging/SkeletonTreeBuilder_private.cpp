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

#include "SkeletonTreeBuilder_private.h"

#include <vcg/complex/allocate.h>

typedef vcg::tri::Allocator<CMeshO> Allocator;

namespace curvatureSkeleton
{

TreeConnectivity::NodeIndices TreeConnectivity::findVertexIndices(CMeshO const& skeleton)
{
	NodeIndices indices;

	for (uint i = 0; i < skeleton.vert.size(); i++)
	{
		indices.insert({ &(skeleton.vert[i]), i});
	}

	return indices;
}

TreeConnectivity::NodeNeighbors TreeConnectivity::findConnectedVertices(CMeshO const& skeleton, NodeIndices const& indices)
{
	NodeNeighbors neighbors;

	for (auto& edge : skeleton.edge)
	{
		auto v0 = indices.at(edge.cV(0));
		auto v1 = indices.at(edge.cV(1));

		auto& v0_neighbors = neighbors.insert( { v0, {} } ).first->second;
		v0_neighbors.insert(v1);
		auto& v1_neighbors = neighbors.insert( { v1, {} } ).first->second;
		v1_neighbors.insert(v0);
	}

	return neighbors;
}

TreeConnectivity::NodeNeighbors TreeConnectivity::findConnectedVertices(CMeshO const& skeleton)
{
	return findConnectedVertices( skeleton, findVertexIndices(skeleton) );
}


IsGraphATree::IsGraphATree(NodeNeighbors const& neighbors)
	: neighbors(neighbors) { }

bool IsGraphATree::isGraphATree(uint start_node, size_t& return_visited_nodes)
{
	auto frontier = FrontierNodes();
	auto visited  = VisitedNodes();
	frontier.push({start_node, start_node});
	while (!frontier.empty())
	{
		auto& head     = frontier.front();
		auto  inserted = visited.insert(head.current_node);
		if (!inserted.second)
		{
			return_visited_nodes = visited.size();
			return false;
		}
		else
		{
			for (auto neighbor : neighbors.at(head.current_node))
			{
				if (neighbor != head.parent_node)
					frontier.push({head.current_node, neighbor});
			}
		}
		frontier.pop();
	}

	return_visited_nodes = visited.size();
	return true;
}


TreeBuilder::TreeBuilder(CMeshO& tree, CMeshO const& skeleton, NodeNeighbors const& neighbors)
	: tree(tree), skeleton(skeleton), neighbors(neighbors) { }

void TreeBuilder::generateTree(uint skeleton_root_vertex)
{
	nodes = Allocator::AddPerVertexAttribute<SkeletonTreeBuilder::SkeletonTreeNode>(
		tree, ATTRIBUTE_TREE_NODE_STRUCT);
	branches = Allocator::AddPerEdgeAttribute<SkeletonTreeBuilder::SkeletonTreeBranch>(
		tree, ATTRIBUTE_TREE_BRANCH_STRUCT);

	Allocator::AddVertex(tree, skeleton.vert[skeleton_root_vertex].cP());
	uint root_node_index = static_cast<uint>(tree.vert.size() - 1);
	{
		auto& root_node           = nodes[root_node_index];
		root_node.skeleton_vertex = skeleton_root_vertex;
	}

	auto frontier = ExpandVertexFrontier();
	for (auto neighbor : neighbors.at(skeleton_root_vertex))
	{
		ExpandVertexData node;
		node.skeleton_parent_vertex  = skeleton_root_vertex;
		node.skeleton_current_vertex = neighbor;
		node.previous_node           = root_node_index;
		node.current_branch.push_back(skeleton_root_vertex);
		frontier.push(node);
	}

	while ( !frontier.empty() )
	{
		auto& head = frontier.front();
		auto& adjacent_vertices = neighbors.at(head.skeleton_current_vertex);
		if (adjacent_vertices.size() == 1)
		{
			closeCurrentBranch(head);
		}
		else if (adjacent_vertices.size() == 2)
		{
			head.current_branch.push_back(head.skeleton_current_vertex);
			auto next_vertex = *(adjacent_vertices.begin());
			if (next_vertex == head.skeleton_parent_vertex)
				next_vertex = *(++adjacent_vertices.begin());

			ExpandVertexData next_node;
			next_node.skeleton_parent_vertex  = head.skeleton_current_vertex;
			next_node.skeleton_current_vertex = next_vertex;
			next_node.previous_node           = head.previous_node;
			next_node.current_branch.swap(head.current_branch);
			frontier.push(next_node);
		}
		else
		{
			auto new_node_index = closeCurrentBranch(head);
			for (auto next_vertex : adjacent_vertices)
			{
				if (next_vertex != head.skeleton_parent_vertex)
				{
					ExpandVertexData next_node;
					next_node.skeleton_parent_vertex  = head.skeleton_current_vertex;
					next_node.skeleton_current_vertex = next_vertex;
					next_node.previous_node           = new_node_index;
					frontier.push(next_node);
				}
			}
		}
		frontier.pop();
	}
}

uint TreeBuilder::closeCurrentBranch(ExpandVertexData node)
{
	node.current_branch.push_back(node.skeleton_current_vertex);

	Allocator::AddVertex(tree, skeleton.vert[node.skeleton_current_vertex].cP());
	uint  new_node_index     = static_cast<uint>(tree.vert.size() - 1);
	auto& new_node           = nodes[new_node_index];
	new_node.skeleton_vertex = node.skeleton_current_vertex;

	Allocator::AddEdge(tree, node.previous_node, new_node_index);
	uint  new_branch_index = static_cast<uint>(tree.edge.size() - 1);
	auto& new_branch       = branches[new_branch_index];

	auto& previous_node = nodes[node.previous_node];
	previous_node.next_branches.push_back(new_branch_index);
	new_node.previous_branch = new_branch_index;
	new_branch.previous_node = node.previous_node;
	new_branch.next_node     = new_node_index;
	new_branch.skeleton_vertices.swap(node.current_branch);

	return new_node_index;
}

SkeletonTreeBuilder::SkeletonTreeNodes TreeBuilder::getTreeNodes(CMeshO const& tree)
{
	auto attribute = Allocator::GetPerVertexAttribute<SkeletonTreeBuilder::SkeletonTreeNode>(tree, ATTRIBUTE_TREE_NODE_STRUCT);
	if (Allocator::IsValidHandle(tree, attribute))
	{
		return attribute;
	}
	else
	{
		throw MLException("Could not find the attribute \"" ATTRIBUTE_TREE_NODE_STRUCT "\" for the given model.");
	}
}

SkeletonTreeBuilder::SkeletonTreeBranches TreeBuilder::getTreeBranches(CMeshO const& tree)
{
	auto attribute = Allocator::GetPerEdgeAttribute<SkeletonTreeBuilder::SkeletonTreeBranch>(tree, ATTRIBUTE_TREE_BRANCH_STRUCT);
	if (Allocator::IsValidHandle(tree, attribute))
	{
		return attribute;
	}
	else
	{
		throw MLException("Could not find the attribute \"" ATTRIBUTE_TREE_BRANCH_STRUCT "\" for the given model.");
	}
}

}
