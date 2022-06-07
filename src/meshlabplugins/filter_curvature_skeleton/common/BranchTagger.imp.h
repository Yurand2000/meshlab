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

#ifndef FILTERCURVATURESKELETON_BRANCH_TAGGER_IMP
#define FILTERCURVATURESKELETON_BRANCH_TAGGER_IMP

#include "SimplifySkeleton.h"

#include <queue>
#include <stack>
#include <vector>
#include <unordered_map>
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/stat.h>
#include <common/mlexception.h>

#define ATTRIBUTE_MESH_TO_SKELETON_INDEX_NAME "skeleton_index"

namespace curvatureSkeleton
{

namespace detail
{
	
template<typename MESH>
struct BranchToColor
{
	typedef typename MESH::VertexType VERTEX;

	VERTEX* start;
	VERTEX* end;
	Scalarm number;
};


template<typename MESH> std::vector<BranchToColor<MESH>> getBranchesToColor(MESH const& tree, MESH& skeleton, std::string attribute_name, bool prioritize_small_values);
template<typename MESH> void paintBranch(MESH& skeleton, BranchToColor<MESH>& branch_to_color, std::string attribute_name, bool prioritize_small_values);
template<typename MESH> void floodUnpaintedBranches(MESH& skeleton, std::string attribute_name, bool prioritize_small_values);

template<typename MESH> int getVertexIndexInMesh(vcg::Point3<Scalarm> point, MESH const& mesh);
template<typename MESH> std::unordered_map<int, int> getTreeToSkeletonAssociations(MESH const& tree, MESH const& skeleton);
template<typename MESH> std::vector<typename MESH::VertexType*> findPath(MESH& mesh, typename MESH::VertexType* start, typename MESH::VertexType* end);
template<typename MESH> std::stack<typename MESH::VertexType const*> getFloodFrontier(MESH const& skeleton, std::string attribute_name);

}

template<typename MESH>
void BranchTagger<MESH>::generateTreeMesh(MESH& tree, MESH const& skeleton, int root_index, Scalarm percentile)
{
	if (root_index < 0 || root_index >= skeleton.vert.size())
		throw MLException("Given index does not represent any valid vertex on the selected mesh.");

	//convert skeleton to SkeletonMesh
	MESH converted_skeleton;
	vcg::tri::Append<MESH, MESH>::MeshCopyConst(converted_skeleton, skeleton);
	vcg::tri::UpdateTopology<MESH>::VertexEdge(converted_skeleton);

	//simplify skeleton
	if (!SimplifySkeleton<MESH>::isMeshConnected(converted_skeleton))
		throw MLException("Given graph mesh must be connected.");

	SimplifySkeleton<MESH>::collapseTwoConnectedVertices(converted_skeleton, root_index);

	vcg::Histogram<Scalarm> histogram;
	vcg::tri::Stat<MESH>::ComputeEdgeLengthHistogram(converted_skeleton, histogram);
	SimplifySkeleton<MESH>::collapseShortEdges(converted_skeleton, root_index, histogram.Percentile(percentile / 100.f));

	//if everything went allright, copy to the given tree mesh
	vcg::tri::Append<MESH, MESH>::MeshCopyConst(tree, converted_skeleton);
	vcg::tri::UpdateTopology<MESH>::VertexEdge(tree);
}





template<typename MESH>
void BranchTagger<MESH>::treeScalarAttributeToSkeleton(MESH& skeleton, MESH const& tree, std::string attribute_name, bool prioritize_small_values)
{
	vcg::tri::UpdateTopology<MESH>::VertexEdge(skeleton);
	vcg::tri::InitVertexIMark(skeleton);
	vcg::tri::InitEdgeIMark(skeleton);

	//prepare attribute
	auto numbers = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(skeleton, attribute_name);
	for (auto& vertex : skeleton.vert)
	{
		if (prioritize_small_values)
			numbers[vertex] = 0;
		else
			numbers[vertex] = std::numeric_limits<Scalarm>().max();
	}

	//set strahler numbers on vertices
	auto branches_to_color = detail::getBranchesToColor(tree, skeleton, attribute_name, prioritize_small_values);
	for (auto& branch : branches_to_color)
	{
		detail::paintBranch(skeleton, branch, attribute_name, prioritize_small_values);
	}
	detail::floodUnpaintedBranches(skeleton, attribute_name, prioritize_small_values);
}

template<typename MESH>
std::vector<detail::BranchToColor<MESH>> detail::getBranchesToColor(MESH const& tree, MESH& skeleton, std::string attribute_name, bool prioritize_small_values)
{
	auto tree_to_skeleton_map = detail::getTreeToSkeletonAssociations(tree, skeleton);
	auto tree_numbers = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(tree, attribute_name);

	std::function<bool(Scalarm const&, Scalarm const&)> less;
	if (prioritize_small_values)
		less = std::less<Scalarm>();
	else
		less = std::greater<Scalarm>();

	std::vector<BranchToColor<MESH>> branches_to_color;
	branches_to_color.reserve(tree.EN());
	for (auto& edge : tree.edge) {
		auto vert0 = &skeleton.vert[tree_to_skeleton_map[edge.V(0)->Index()]];
		auto vert1 = &skeleton.vert[tree_to_skeleton_map[edge.V(1)->Index()]];
		Scalarm num0  = tree_numbers[edge.V(0)];
		Scalarm num1  = tree_numbers[edge.V(1)];
		if ( less(num0, num1) )
			branches_to_color.push_back({vert0, vert1, num0});
		else
			branches_to_color.push_back({vert1, vert0, num1});
	}

	return branches_to_color;
}


template<typename MESH>
std::unordered_map<int, int> detail::getTreeToSkeletonAssociations(MESH const& tree, MESH const& skeleton)
{
	std::unordered_map<int, int> tree_to_skeleton_map;
	for (auto& vertex : tree.vert)
	{
		tree_to_skeleton_map.emplace( vertex.Index(), detail::getVertexIndexInMesh(vertex.P(), skeleton) );
	}

	return tree_to_skeleton_map;
}

template<typename MESH>
int detail::getVertexIndexInMesh(vcg::Point3<Scalarm> point, MESH const& mesh)
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


template<typename MESH>
void detail::paintBranch(MESH& skeleton, BranchToColor<MESH>& branch_to_color, std::string attribute_name, bool prioritize_small_values)
{
	std::function<bool(Scalarm const&, Scalarm const&)> less;
	if (prioritize_small_values)
		less = std::less<Scalarm>();
	else
		less = std::greater<Scalarm>();

	auto numbers = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(skeleton, attribute_name);
	auto path = findPath(skeleton, branch_to_color.start, branch_to_color.end);
	for (auto* vertex : path)
	{
		auto& number = numbers[vertex];
		if ( less(number, branch_to_color.number) )
			number = branch_to_color.number;
	}
}

template<typename MESH>
std::vector<typename MESH::VertexType*> detail::findPath(MESH& mesh, typename MESH::VertexType* start, typename MESH::VertexType* end)
{
	using StarVector = std::vector<typename MESH::VertexType*>;

	vcg::tri::UnMarkAll(mesh);
	std::vector<typename MESH::VertexType*> parent;
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


template<typename MESH>
void detail::floodUnpaintedBranches(MESH& skeleton, std::string attribute_name, bool prioritize_small_values)
{
	auto null_number = (prioritize_small_values) ? 0 : std::numeric_limits<Scalarm>::max();

	auto numbers = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(skeleton, attribute_name);
	auto flood_frontier = detail::getFloodFrontier(skeleton, attribute_name);

	while ( !flood_frontier.empty() )
	{
		auto* vertex = flood_frontier.top();
		flood_frontier.pop();

		std::vector<typename MESH::VertexType*> star;
		vcg::edge::VVStarVE(vertex, star);

		auto current_number = numbers[vertex];
		for (auto* adj : star)
		{
			auto& number = numbers[adj];
			if ( number == null_number )
			{
				number = current_number;
				flood_frontier.push(adj);
			}
		}
	}
}

template<typename MESH>
std::stack<typename MESH::VertexType const*> detail::getFloodFrontier(MESH const& skeleton, std::string attribute_name)
{
	auto numbers = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(skeleton, attribute_name);
	std::stack<typename MESH::VertexType const*> frontier;
	for (auto& vertex : skeleton.vert)
	{
		if ( (numbers[vertex] != 0) && (vcg::edge::VEDegree<typename MESH::EdgeType>(&vertex) > 2) )
		{
			frontier.push(&vertex);
		}
	}

	return frontier;
}




template<typename MESH>
void BranchTagger<MESH>::skeletonScalarAttributeToOriginalMesh(MESH& mesh, MESH& skeleton, std::string attribute_name)
{
	auto skeleton_numbers = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(skeleton, attribute_name);
	auto original_numbers = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(mesh, attribute_name);
	auto original_to_skeleton = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(mesh, ATTRIBUTE_MESH_TO_SKELETON_INDEX_NAME);

	for (auto& vert : mesh.vert)
	{
		auto skeleton_index = static_cast<int>( original_to_skeleton[vert] );
		if (skeleton_index >= 0)
			original_numbers[vert] = skeleton_numbers[skeleton_index];
	}
}

template<typename MESH>
std::vector<typename BranchTagger<MESH>::Color> BranchTagger<MESH>::generateColorsFromAttribute(MESH const& mesh, std::string attribute_name)
{
	auto tree_numbers = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(mesh, attribute_name);

	uint min = 1, max = 1;
	for (int i = 0; i < mesh.VN(); i++)
	{
		if (tree_numbers[i] > max)
			max = tree_numbers[i];
	}

	uint range  = max - min + 1;
	auto colors = std::vector<Color>(range + min);
	for (uint i = 0; i < range; i++)
	{
		colors[i + min] = Color::Scatter(range, i, 0.6f, 1.f);
	}

	return colors;
}

template<typename MESH>
void BranchTagger<MESH>::colorizeByAttribute(MESH& mesh, std::vector<Color> const& colors, std::string attribute_name)
{
	auto numbers = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(mesh, attribute_name);
	for (int i = 0; i < mesh.VN(); i++)
	{
		auto& vertex = mesh.vert[i];
		auto  number = numbers[i];

		if (number < colors.size())
			vertex.C() = colors[number];
	}
}

}

#endif // FILTERCURVATURESKELETON_BRANCH_TAGGER_IMP
