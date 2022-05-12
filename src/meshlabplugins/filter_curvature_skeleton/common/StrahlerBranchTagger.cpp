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



void StrahlerBranchTagger::generateTreeMesh(SkeletonMesh& tree, CMeshO const& skeleton, int root_index, Scalarm percentile)
{
	if (root_index < 0 || root_index >= skeleton.vert.size())
		throw MLException("Given index does not represent any valid vertex on the selected mesh.");

	//convert skeleton to SkeletonMesh
	SkeletonMesh converted_skeleton;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_skeleton, skeleton);
	SkeletonMeshTopology::VertexEdge(converted_skeleton);

	//simplify skeleton
	if (!SimplifySkeleton::isMeshConnected(converted_skeleton))
		throw MLException("Given graph mesh must be connected.");

	if ( vcg::edge::VEDegree<SkeletonEdge>(&converted_skeleton.vert[root_index]) != 1 )
		throw MLException("Given root node must be a border vertex.");


	SimplifySkeleton::collapseTwoConnectedVertices(converted_skeleton);

	vcg::Histogram<Scalarm> histogram;
	vcg::tri::Stat<SkeletonMesh>::ComputeEdgeLengthHistogram(converted_skeleton, histogram);
	SimplifySkeleton::collapseShortEdges(converted_skeleton, root_index, histogram.Percentile(percentile / 100.f));

	SimplifySkeleton::collapseTwoConnectedVertices(converted_skeleton);

	//if everything went allright, copy to the given tree mesh
	SkeletonToSkeletonAppend::MeshCopyConst(tree, converted_skeleton);
	SkeletonMeshTopology::VertexEdge(tree);
}





struct BranchToColor
{
	SkeletonVertex* start;
	SkeletonVertex* end;
	uint            number;
};

static std::vector<BranchToColor> getBranchesToColor(SkeletonMesh const& tree, SkeletonMesh& skeleton);
static void paintBranch(SkeletonMesh& skeleton, BranchToColor& branch_to_color);
static void floodUnpaintedBranches(SkeletonMesh& skeleton);

void StrahlerBranchTagger::strahlerNumbersToSkeleton(CMeshO& skeleton, SkeletonMesh const& tree, int root_index)
{
	//convert skeleton to SkeletonMesh
	SkeletonMesh converted_skeleton;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_skeleton, skeleton);
	SkeletonMeshTopology::VertexEdge(converted_skeleton);
	vcg::tri::InitVertexIMark(converted_skeleton);
	vcg::tri::InitEdgeIMark(converted_skeleton);
	SkeletonAllocator::AddPerVertexAttribute<Scalarm>(converted_skeleton, ATTRIBUTE_STRAHLER_NUMBER);

	//set strahler numbers on vertices
	auto branches_to_color = getBranchesToColor(tree, converted_skeleton);
	for (auto& branch : branches_to_color)
	{
		paintBranch(converted_skeleton, branch);
	}
	floodUnpaintedBranches(converted_skeleton);

	//reconvert the skeleton to CMeshO
	auto skeleton_numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(skeleton, ATTRIBUTE_STRAHLER_NUMBER);
	SkeletonToCMeshOAppend::MeshCopyConst(skeleton, converted_skeleton);
}

static std::unordered_map<int, int> getTreeToSkeletonAssociations(SkeletonMesh const& tree, SkeletonMesh const& skeleton);
static std::vector<BranchToColor> getBranchesToColor(SkeletonMesh const& tree, SkeletonMesh& skeleton)
{
	auto tree_to_skeleton_map = getTreeToSkeletonAssociations(tree, skeleton);
	auto tree_numbers = SkeletonAllocator::GetPerVertexAttribute<Scalarm>(tree, ATTRIBUTE_STRAHLER_NUMBER);

	std::vector<BranchToColor> branches_to_color;
	branches_to_color.reserve(tree.EN());
	for (auto& edge : tree.edge) {
		auto vert0 = &skeleton.vert[tree_to_skeleton_map[edge.V(0)->Index()]];
		auto vert1 = &skeleton.vert[tree_to_skeleton_map[edge.V(1)->Index()]];
		uint num0  = tree_numbers[edge.V(0)];
		uint num1  = tree_numbers[edge.V(1)];
		if (num0 < num1)
			branches_to_color.push_back({vert0, vert1, num0});
		else
			branches_to_color.push_back({vert1, vert0, num1});
	}

	return branches_to_color;
}



template <typename MESH> static int getVertexIndexInMesh(vcg::Point3<Scalarm> point, MESH const& mesh);
std::unordered_map<int, int> getTreeToSkeletonAssociations(SkeletonMesh const& tree, SkeletonMesh const& skeleton)
{
	std::unordered_map<int, int> tree_to_skeleton_map;
	for (auto& vertex : tree.vert)
	{
		tree_to_skeleton_map.emplace( vertex.Index(), getVertexIndexInMesh(vertex.P(), skeleton) );
	}

	return tree_to_skeleton_map;
}

template<typename MESH>
int getVertexIndexInMesh(vcg::Point3<Scalarm> point, MESH const& mesh)
{
	Scalarm min_sqr_dist = std::numeric_limits<Scalarm>::max();
	int index = -1;
	for (auto& vertex : mesh.vert)
	{
		auto sqr_dist = (point - vertex.cP()).SquaredNorm();
		if (sqr_dist < min_sqr_dist)
		{
			min_sqr_dist = sqr_dist;
			index        = vertex.Index();
		}
	}
	return index;
}



static std::vector<SkeletonVertex*> findPath(SkeletonMesh& mesh, SkeletonVertex* start, SkeletonVertex* end);
void paintBranch(SkeletonMesh& skeleton, BranchToColor& branch_to_color)
{
	auto numbers = SkeletonAllocator::GetPerVertexAttribute<Scalarm>(skeleton, ATTRIBUTE_STRAHLER_NUMBER);
	auto path = findPath(skeleton, branch_to_color.start, branch_to_color.end);
	for (auto* vertex : path)
	{
		auto& number = numbers[vertex];
		if (number < branch_to_color.number)
			number = branch_to_color.number;
	}
}

std::vector<SkeletonVertex*> findPath(SkeletonMesh& mesh, SkeletonVertex* start, SkeletonVertex* end)
{
	using StarVector = std::vector<SkeletonVertex*>;

	vcg::tri::UnMarkAll(mesh);
	std::vector<SkeletonVertex*> parent;
	std::stack<StarVector>       last_pos;

	parent.push_back(start);
	last_pos.emplace();
	vcg::edge::VVStarVE(start, last_pos.top());

	do
	{
		auto* current  = parent.back();
		vcg::tri::Mark(mesh, current);
		auto& curr_adj = last_pos.top();
		if (curr_adj.empty())
		{
			parent.pop_back();
			last_pos.pop();
		}
		else
		{
			auto* next = curr_adj.back();
			curr_adj.pop_back();
			if ( !vcg::tri::IsMarked(mesh, next) )
			{
				parent.push_back(next);
				last_pos.emplace();
				vcg::edge::VVStarVE(next, last_pos.top());
			}
		}
	}
	while ( !parent.empty() && parent.back() != end );

	if (parent.empty())
		throw MLException("Mesh is not connected!");
	
	parent.push_back(end);
	return parent;
}



static std::stack<SkeletonVertex const*> getFloodFrontier(SkeletonMesh const& skeleton);
void floodUnpaintedBranches(SkeletonMesh& skeleton)
{
	auto numbers = SkeletonAllocator::GetPerVertexAttribute<Scalarm>(skeleton, ATTRIBUTE_STRAHLER_NUMBER);
	auto flood_frontier = getFloodFrontier(skeleton);

	do
	{
		auto* vertex = flood_frontier.top();
		flood_frontier.pop();

		std::vector<SkeletonVertex*> star;
		vcg::edge::VVStarVE(vertex, star);

		auto current_number = numbers[vertex];
		for (auto* adj : star)
		{
			auto& number = numbers[adj];
			if ( number == 0 )
			{
				number = current_number;
				flood_frontier.push(adj);
			}
		}
	}
	while ( !flood_frontier.empty() );
}

std::stack<SkeletonVertex const*> getFloodFrontier(SkeletonMesh const& skeleton)
{
	auto numbers = SkeletonAllocator::GetPerVertexAttribute<Scalarm>(skeleton, ATTRIBUTE_STRAHLER_NUMBER);
	std::stack<SkeletonVertex const*> frontier;
	for (auto& vertex : skeleton.vert)
	{
		if ( (numbers[vertex] != 0) && (vcg::edge::VEDegree<SkeletonEdge>(&vertex) > 2) )
		{
			frontier.push(&vertex);
		}
	}

	return frontier;
}





void StrahlerBranchTagger::strahlerNumbersToOriginalMesh(CMeshO& mesh, CMeshO& skeleton)
{
	auto skeleton_numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(skeleton, ATTRIBUTE_STRAHLER_NUMBER);
	auto original_numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(mesh, ATTRIBUTE_STRAHLER_NUMBER);
	auto original_to_skeleton = CMeshOAllocator::GetPerVertexAttribute<uint>(mesh, ATTRIBUTE_MESH_TO_SKELETON_INDEX_NAME);

	for (int i = 0; i < mesh.VN(); i++)
	{
		original_numbers[i] = skeleton_numbers[ original_to_skeleton[i] ];
	}
}

}
