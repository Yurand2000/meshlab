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
#include <common/mlexception.h>

namespace curvatureSkeleton
{

typedef vcg::tri::UpdateTopology<SkeletonMesh>       SkeletonMeshTopology;
typedef vcg::tri::Allocator<CMeshO>                  CMeshOAllocator;
typedef vcg::tri::Allocator<SkeletonMesh>            SkeletonAllocator;
typedef vcg::tri::Append<CMeshO, SkeletonMesh>       SkeletonToCMeshOAppend;
typedef vcg::tri::Append<SkeletonMesh, CMeshO>       CMeshOToSkeletonAppend;

template <typename MESH> static int getVertexIndexInMesh(vcg::Point3<Scalarm> point, MESH const& mesh);
template <typename MESH> static bool isVertexInMesh(vcg::Point3<Scalarm> point, MESH const& mesh);
static std::unordered_set<SkeletonVertex const*> findPath(SkeletonMesh const& mesh, int start, int end);

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
			auto v_index = vert->Index();
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
	
	std::unordered_map<int, int> tree_to_skeleton_map;
	for (auto& vertex : tree.vert)
	{
		tree_to_skeleton_map.emplace( vertex.Index(), getVertexIndexInMesh(vertex.P(), skeleton));
	}

	struct BranchToColor
	{
		int start;
		int end;
		uint number;
	};
	std::vector<BranchToColor> branches_to_color;
	branches_to_color.reserve( tree.EN() );
	for (auto& edge : tree.edge)
	{
		auto vert0 = tree_to_skeleton_map[edge.V(0)->Index()];
		auto vert1 = tree_to_skeleton_map[edge.V(1)->Index()];
		uint num0  = tree_numbers[edge.V(0)->Index()];
		uint num1  = tree_numbers[edge.V(1)->Index()];
		if (num0 < num1)
			branches_to_color.push_back( {vert0, vert1, num0} );
		else
			branches_to_color.push_back( {vert1, vert0, num1} );
	}

	for (auto& branch : branches_to_color)
	{
		auto path = findPath(converted_skeleton, branch.start, branch.end);
		for (auto* vertex : path)
		{
			auto& number = numbers[vertex->Index()];
			if (number < branch.number)
				number = branch.number;
		}
	}

	std::stack<SkeletonVertex const*> flood_frontier;
	for (auto& vertex : converted_skeleton.vert)
	{
		if ( numbers[vertex.Index()] != 0 )
			flood_frontier.push(&vertex);
	}

	do
	{
		auto* vertex = flood_frontier.top();
		flood_frontier.pop();

		std::vector<SkeletonVertex*> star;
		vcg::edge::VVStarVE(vertex, star);

		auto current_number = numbers[vertex->Index()];
		for (auto* adj : star)
		{
			auto& number = numbers[adj->Index()];
			if ( number == 0 )
			{
				number = current_number;
				flood_frontier.push(adj);
			}
		}
	}
	while ( !flood_frontier.empty() );

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

template<typename MESH>
int getVertexIndexInMesh(vcg::Point3<Scalarm> point, MESH const& mesh)
{
	for (int i = 0; i < mesh.vert.size(); i++)
	{
		auto& node = mesh.vert[i];
		if ((point - node.cP()).SquaredNorm() < 0.0001f)
			return i;
	}
	return -1;
}
template<typename MESH>
bool isVertexInMesh(vcg::Point3<Scalarm> point, MESH const& mesh)
{
	return getVertexIndexInMesh<MESH>(point, mesh) != -1;
}

std::unordered_set<SkeletonVertex const*> findPath(SkeletonMesh const& mesh, int start, int end)
{
	int current = start;
	std::stack<int> parent;
	parent.push(start);
	std::stack<int> last_pos;
	last_pos.push(0);

	do
	{
		if (current == end)
		{
			parent.push(end);
		}
		else
		{			
			auto* vert = &mesh.vert[current];
			std::vector<SkeletonVertex*> star;
			vcg::edge::VVStarVE(vert, star);

			auto last = last_pos.top();
			if (last == star.size())
			{
				if (parent.empty())
					throw MLException("Mesh is not connected!");

				current = parent.top();
				parent.pop();
				last_pos.pop();
			}
			else
			{
				auto next = star[last]->Index();
				last_pos.top()++;
				if (next != parent.top())
				{
					parent.push(current);
					current = next;
					last_pos.push(0);
				}
			}
		}
	}
	while ( parent.top() != end );

	std::unordered_set<SkeletonVertex const*> path;
	do
	{
		path.insert( &mesh.vert[parent.top()] );
		parent.pop();
	}
	while( !parent.empty() );

	return path;
}

}
