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

#include "common/additionalAttributeNames.h"

#include <queue>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <vcg/complex/complex.h>

namespace curvatureSkeleton
{

typedef vcg::tri::UpdateTopology<SkeletonMesh>       SkeletonMeshTopology;
typedef vcg::tri::Allocator<CMeshO>                  CMeshOAllocator;
typedef vcg::tri::Allocator<SkeletonMesh>            SkeletonAllocator;
typedef vcg::tri::Append<CMeshO, SkeletonMesh>       SkeletonToCMeshOAppend;
typedef vcg::tri::Append<SkeletonMesh, CMeshO>       CMeshOToSkeletonAppend;

static std::unordered_map<SkeletonVertex const*, int> verticesToIndices(SkeletonMesh const& skeleton);
static int getVertexIndexInMesh(vcg::Point3<Scalarm> point, SkeletonMesh const& mesh);
static bool isVertexInMesh(vcg::Point3<Scalarm> point, SkeletonMesh const& mesh);

void calculateStrahlerNumbers(SkeletonMesh& tree, int root_index)
{
	auto attribute = SkeletonAllocator::GetPerVertexAttribute<uint>(tree, ATTRIBUTE_STRAHLER_NUMBER);
	for (int i = 0; i < tree.vert.size(); i++)
		attribute[i] = 1;

	struct StrahlerNode
	{
		int node;
		int parent;
	};
	std::stack<StrahlerNode> tree_reverse;

	std::queue<int> frontier;
	std::unordered_set<int> visited;
	auto skelton_to_index = verticesToIndices(tree);
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

void strahlerNumbersToSkeleton(CMeshO& skeleton, SkeletonMesh const& tree, int root_index)
{
	SkeletonMesh converted_skeleton;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_skeleton, skeleton);
	SkeletonMeshTopology::VertexEdge(converted_skeleton);
	auto numbers = SkeletonAllocator::AddPerVertexAttribute<uint>(converted_skeleton, ATTRIBUTE_STRAHLER_NUMBER);
	auto tree_numbers = SkeletonAllocator::GetPerVertexAttribute<uint>(tree, ATTRIBUTE_STRAHLER_NUMBER);

	auto skelton_to_index = verticesToIndices(converted_skeleton);

	auto* root = &converted_skeleton.vert[root_index];
	std::unordered_set<SkeletonVertex*> visited;

	typedef std::pair<int, uint> queue_pair;
	auto less_fun = [](std::pair<int, uint> const& l, std::pair<int, uint> const& r) -> bool
	{
		return r.second < l.second;
	};
	auto frontier = std::priority_queue<queue_pair, std::vector<queue_pair>, decltype(less_fun)>(less_fun);
	for (int i = 0; i < converted_skeleton.vert.size(); i++)
	{
		auto* vert = &converted_skeleton.vert[i];
		if (vcg::edge::VEDegree<SkeletonEdge>(vert) == 1 && root != vert && isVertexInMesh(vert->P(), tree))
			frontier.emplace(i, 1);
	}

	do
	{
		auto  index  = frontier.top().first;
		auto* vert   = &converted_skeleton.vert[index];
		auto  number = frontier.top().second;
		frontier.pop();
		if (visited.count(vert) == 1)
			continue;

		visited.insert(vert);

		std::vector<SkeletonVertex*> adj_verts;
		vcg::edge::VVStarVE(vert, adj_verts);
		if (adj_verts.size() > 2)
		{
			auto tree_index = getVertexIndexInMesh(vert->P(), tree);
			if (tree_index != -1)
				number = tree_numbers[tree_index];
		}

		for (auto* adj : adj_verts)
		{
			if (visited.count(adj) == 0)
				frontier.emplace(skelton_to_index[adj], number);
		}

		numbers[index] = number;
	}
	while ( !frontier.empty() );

	auto skeleton_numbers = CMeshOAllocator::GetPerVertexAttribute<uint>(skeleton, ATTRIBUTE_STRAHLER_NUMBER);
	SkeletonToCMeshOAppend::MeshCopyConst(skeleton, converted_skeleton);
}

void strahlerNumbersToOriginalMesh(CMeshO& mesh, CMeshO& skeleton)
{
	auto skeleton_numbers = CMeshOAllocator::GetPerVertexAttribute<uint>(skeleton, ATTRIBUTE_STRAHLER_NUMBER);
	auto original_numbers = CMeshOAllocator::GetPerVertexAttribute<uint>(mesh, ATTRIBUTE_STRAHLER_NUMBER);
	auto original_to_skeleton = CMeshOAllocator::GetPerVertexAttribute<uint>(mesh, ATTRIBUTE_MESH_TO_SKELETON_INDEX_NAME);
	for (int i = 0; i < mesh.VN(); i++)
	{
		original_numbers[i] = skeleton_numbers[ original_to_skeleton[i] ];
	}
}

std::unordered_map<SkeletonVertex const*, int> verticesToIndices(SkeletonMesh const& skeleton)
{
	std::unordered_map<SkeletonVertex const*, int> skelton_to_index;
	for (int i = 0; i < skeleton.vert.size(); i++)
	{
		auto* vertex = &skeleton.vert[i];
		skelton_to_index.emplace(vertex, i);
	}
	return skelton_to_index;
}

int getVertexIndexInMesh(vcg::Point3<Scalarm> point, SkeletonMesh const& mesh)
{
	for (int i = 0; i < mesh.vert.size(); i++)
	{
		auto& node = mesh.vert[i];
		if ((point - node.cP()).SquaredNorm() < 0.0001f)
			return i;
	}
	return -1;
}

bool isVertexInMesh(vcg::Point3<Scalarm> point, SkeletonMesh const& mesh)
{
	return getVertexIndexInMesh(point, mesh) != -1;
}

}
