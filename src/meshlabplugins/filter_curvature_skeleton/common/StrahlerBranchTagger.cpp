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

#include "additionalAttributeNames.h"
#include "SimplifySkeleton.h"

#include <queue>
#include <stack>
#include <vector>
#include <unordered_map>
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/stat.h>
#include <common/mlexception.h>

namespace curvatureSkeleton
{

typedef vcg::tri::UpdateTopology<SkeletonMesh>       SkeletonMeshTopology;
typedef vcg::tri::Allocator<CMeshO>                  CMeshOAllocator;
typedef vcg::tri::Allocator<SkeletonMesh>            SkeletonAllocator;
typedef vcg::tri::Append<CMeshO, SkeletonMesh>       SkeletonToCMeshOAppend;
typedef vcg::tri::Append<SkeletonMesh, CMeshO>       CMeshOToSkeletonAppend;
typedef vcg::tri::Append<SkeletonMesh, SkeletonMesh> SkeletonToSkeletonAppend;

template <typename MESH> static int getVertexIndexInMesh(vcg::Point3<Scalarm> point, MESH const& mesh);
template <typename MESH> static bool isVertexInMesh(vcg::Point3<Scalarm> point, MESH const& mesh);
static std::vector<SkeletonVertex const*> findPath(SkeletonMesh const& mesh, int start, int end);

void StrahlerBranchTagger::generateTreeMesh(SkeletonMesh& tree, CMeshO const& skeleton, int root_index, Scalarm percentile)
{
	if (root_index < 0 || root_index >= skeleton.vert.size())
		throw MLException("Given index does not represent any valid vertex on the selected mesh.");

	SkeletonMesh converted_skeleton;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_skeleton, skeleton);
	SkeletonMeshTopology::VertexEdge(converted_skeleton);

	if (!SimplifySkeleton::isMeshConnected(converted_skeleton))
		throw MLException("Given graph mesh must be connected.");

	SimplifySkeleton::collapseTwoConnectedVertices(converted_skeleton);

	vcg::Histogram<Scalarm> histogram;
	vcg::tri::Stat<SkeletonMesh>::ComputeEdgeLengthHistogram(converted_skeleton, histogram);
	SimplifySkeleton::collapseShortEdges(converted_skeleton, root_index, histogram.Percentile(percentile / 100.f));

	SimplifySkeleton::collapseTwoConnectedVertices(converted_skeleton);

	SkeletonToSkeletonAppend::MeshCopyConst(tree, converted_skeleton);
	SkeletonMeshTopology::VertexEdge(tree);
}

void StrahlerBranchTagger::calculateStrahlerNumbers(SkeletonMesh& tree, int root_index)
{
	auto attribute = SkeletonAllocator::GetPerVertexAttribute<uint>(tree, ATTRIBUTE_STRAHLER_NUMBER);
	for (int i = 0; i < tree.VN(); i++)
		attribute[i] = 1;

	struct StrahlerNode
	{
		SkeletonVertex* node;
		SkeletonVertex* parent;
	};
	std::stack<StrahlerNode> tree_reverse;

	std::queue<SkeletonVertex*> frontier;
	vcg::tri::UnMarkAll(tree);
	frontier.push(&tree.vert[root_index]);
	do
	{
		auto* vertex = frontier.front();
		vcg::tri::Mark(tree, vertex);
		frontier.pop();

		std::vector<SkeletonVertex*> verts;
		vcg::edge::VVStarVE(vertex, verts);

		for (auto* adj : verts)
		{
			if (!vcg::tri::IsMarked(tree, adj))
			{
				tree_reverse.push({adj, vertex});
				frontier.push(adj);
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

void StrahlerBranchTagger::strahlerNumbersToSkeleton(CMeshO& skeleton, SkeletonMesh const& tree, int root_index)
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

	std::stack<SkeletonVertex const*> flood_frontier;
	for (auto& branch : branches_to_color)
	{
		auto path = findPath(converted_skeleton, branch.start, branch.end);
		for (auto* vertex : path)
		{
			flood_frontier.push(vertex);
			auto& number = numbers[vertex->Index()];
			if (number < branch.number)
				number = branch.number;
		}
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

void StrahlerBranchTagger::strahlerNumbersToOriginalMesh(CMeshO& mesh, CMeshO& skeleton)
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

std::vector<SkeletonVertex const*> findPath(SkeletonMesh const& mesh, int start, int end)
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

	std::vector<SkeletonVertex const*> path;
	do
	{
		path.push_back( &mesh.vert[parent.top()] );
		parent.pop();
	}
	while( !parent.empty() );

	return path;
}

}
