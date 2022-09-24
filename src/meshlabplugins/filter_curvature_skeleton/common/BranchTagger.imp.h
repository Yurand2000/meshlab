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
#include "Utils.h"

#include <queue>
#include <stack>
#include <vector>
#include <unordered_map>
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/stat.h>
#include <common/mlexception.h>

namespace curvatureSkeleton
{

namespace detail
{
	
template<typename MESH>
struct Edge
{
	typedef typename MESH::VertexType VERTEX;

	VERTEX* start;
	VERTEX* end;
	Scalarm number;
};

inline Scalarm getNullValue(bool ascending);
inline std::function<bool(Scalarm const&, Scalarm const&)> getCompareFunction(bool ascending);
template<typename MESH> std::vector<Edge<MESH>> getBranchesToColor(MESH& skeleton, MESH const& tree, std::string const& attribute_name, bool ascending);
template<typename MESH> void paintEdge(MESH& skeleton, Edge<MESH>& branch_to_color, std::string const& attribute_name, bool ascending);
template<typename MESH> void floodUnassignedEdges(MESH& skeleton, std::string const& attribute_name, bool ascending);

template<typename MESH> std::vector<typename MESH::VertexType*> findPath(MESH& mesh, typename MESH::VertexType* start, typename MESH::VertexType* end);

template<typename MESH> std::unordered_map<int, int> getTreeToSkeletonAssociations(MESH const& tree, MESH const& skeleton);

}






template<typename MESH>
void BranchTagger<MESH>::generateTreeMesh(MESH& tree, MESH const& skeleton, int root_index, Scalarm min_length, Scalarm percentile)
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

	//collapse edges shorter than the minimum edge length
	SimplifySkeleton<MESH>::collapseTwoConnectedVertices(converted_skeleton, root_index);
	SimplifySkeleton<MESH>::collapseShortEdges(converted_skeleton, root_index, min_length);

	//collapse edges shorter than the given percentile
	vcg::Histogram<Scalarm> histogram;
	vcg::tri::Stat<MESH>::ComputeEdgeLengthHistogram(converted_skeleton, histogram);
	SimplifySkeleton<MESH>::collapseShortEdges(converted_skeleton, root_index, histogram.Percentile(percentile / 100.f));

	//if everything went allright, copy to the given tree mesh
	vcg::tri::Append<MESH, MESH>::MeshCopyConst(tree, converted_skeleton);
	vcg::tri::UpdateTopology<MESH>::VertexEdge(tree);
}






template<typename MESH>
void BranchTagger<MESH>::copyAttributeTreeToSkeleton(MESH& skeleton, MESH& tree, int tree_root_index, std::string const& attribute_name, bool ascending)
{
	// prepare attribute
	auto skeleton_attribute = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(skeleton, attribute_name);
	for (auto& vertex : skeleton.vert)
		skeleton_attribute[vertex] = detail::getNullValue(ascending);

	//copy attribute
	auto branches_to_color = detail::getBranchesToColor(skeleton, tree, attribute_name, ascending);
	for (auto& branch : branches_to_color)
	{
		detail::paintEdge(skeleton, branch, attribute_name, ascending);
	}
	detail::floodUnassignedEdges(skeleton, attribute_name, ascending);
}

template<typename MESH>
std::vector<detail::Edge<MESH>> detail::getBranchesToColor(MESH& skeleton, MESH const& tree, std::string const& attribute_name, bool ascending)
{
	auto attribute = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(tree, attribute_name);
	auto less = detail::getCompareFunction(ascending);
	auto tree_to_skeleton = detail::getTreeToSkeletonAssociations(tree, skeleton);

	std::vector<detail::Edge<MESH>> branches_to_color;
	branches_to_color.reserve(tree.EN());
	for (auto& edge : tree.edge)
	{
		if ( !edge.IsD() )
		{
			auto vert0   = &skeleton.vert[ tree_to_skeleton[edge.V(0)->Index()] ];
			auto vert1   = &skeleton.vert[ tree_to_skeleton[edge.V(1)->Index()] ];
			Scalarm num0 = attribute[ edge.V(0) ];
			Scalarm num1 = attribute[ edge.V(1) ];
			if ( less(num0, num1) )
				branches_to_color.push_back( {vert0, vert1, num0} );
			else
				branches_to_color.push_back( {vert1, vert0, num1} );
		}
	}

	return branches_to_color;
}

template<typename MESH>
void detail::paintEdge(MESH& skeleton, Edge<MESH>& branch_to_color, std::string const& attribute_name, bool ascending)
{
	auto less = detail::getCompareFunction(ascending);

	auto numbers = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(skeleton, attribute_name);
	auto path = detail::findPath<MESH>(skeleton, branch_to_color.start, branch_to_color.end);
	for (auto* vertex : path)
	{
		auto& number = numbers[vertex];
		if ( less(number, branch_to_color.number) )
			number = branch_to_color.number;
	}
}

inline Scalarm detail::getNullValue(bool ascending)
{
	if (ascending)
		return std::numeric_limits<Scalarm>().max();
	else
		return std::numeric_limits<Scalarm>().min();
}

inline std::function<bool(Scalarm const&, Scalarm const&)> detail::getCompareFunction(bool ascending)
{
	if (ascending)
		return std::greater<Scalarm>();
	else
		return std::less<Scalarm>();
}

template<typename MESH>
std::unordered_map<int, int> detail::getTreeToSkeletonAssociations(MESH const& tree, MESH const& skeleton)
{
	std::unordered_map<int, int> tree_to_skeleton_map;
	for (auto& vertex : tree.vert)
	{
		tree_to_skeleton_map.emplace( vertex.Index(), Utils<MESH>::getVertexIndexInMesh(vertex.P(), skeleton) );
	}

	return tree_to_skeleton_map;
}

template<typename MESH>
std::vector<typename MESH::VertexType*> detail::findPath(MESH& mesh, typename MESH::VertexType* start, typename MESH::VertexType* end)
{
	using StarVector = std::vector<SkeletonMesh::VertexType*>;

	SkeletonMesh converted_mesh;
	vcg::tri::Append<SkeletonMesh, MESH>::MeshCopyConst(converted_mesh, mesh);
	vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(converted_mesh);
	auto* converted_start = &converted_mesh.vert[start->Index()];
	auto* converted_end = &converted_mesh.vert[end->Index()];

	//setup
	vcg::tri::UnMarkAll(converted_mesh);
	std::stack<SkeletonMesh::VertexType*> parent;
	std::stack<StarVector>                last_pos;

	parent.push(converted_start);
	last_pos.emplace();
	vcg::edge::VVStarVE(converted_start, last_pos.top());

	//explore tree to find the path
	do
	{
		auto* current  = parent.top();
		vcg::tri::Mark(converted_mesh, current);
		auto& curr_adj = last_pos.top();
		if (curr_adj.empty())
		{
			parent.pop();
			last_pos.pop();
		}
		else
		{
			auto* next = curr_adj.back();
			curr_adj.pop_back();
			if ( !vcg::tri::IsMarked(converted_mesh, next) )
			{
				parent.push(next);
				last_pos.emplace();
				vcg::edge::VVStarVE(next, last_pos.top());
			}
		}
	}
	while ( !parent.empty() && parent.top() != converted_end );

	if (parent.empty())
		throw MLException("Mesh is not connected!");

	//copy path to vector
	std::vector<typename MESH::VertexType*> mesh_parent;
	mesh_parent.reserve( parent.size() );
	do
	{
		auto vertex = &mesh.vert[ parent.top()->Index() ];
		parent.pop();

		mesh_parent.push_back(vertex);
	}
	while ( !parent.empty() );
	return mesh_parent;
}


template<typename MESH>
void detail::floodUnassignedEdges(MESH& mesh, std::string const& attribute_name, bool ascending)
{
	auto null_value = detail::getNullValue(ascending);

	SkeletonMesh converted_mesh;
	vcg::tri::Append<SkeletonMesh, MESH>::MeshCopyConst(converted_mesh, mesh);
	vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(converted_mesh);

	auto attribute = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(mesh, attribute_name);

	//get assigned vertices
	std::stack<SkeletonMesh::VertexType const*> flood_frontier;
	for (auto& vertex : converted_mesh.vert)
	{
		if ( attribute[vertex.Index()] != null_value )
			flood_frontier.push(&vertex);
	}

	//flood to unassigned vertices
	while ( !flood_frontier.empty() )
	{
		auto* vertex = flood_frontier.top();
		flood_frontier.pop();

		std::vector<SkeletonMesh::VertexType*> star;
		vcg::edge::VVStarVE(vertex, star);

		auto current_number = attribute[vertex->Index()];
		for (auto* adj : star)
		{
			auto& number = attribute[adj->Index()];
			if ( number == null_value )
			{
				number = current_number;
				flood_frontier.push(adj);
			}
		}
	}
}





template<typename MESH>
void BranchTagger<MESH>::copyAttributeUsingAdjacency(MESH& start, MESH& end, std::string const& attribute_name, std::string const& adjacency_attribute_name)
{
	auto start_numbers = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(start, attribute_name);
	auto end_numbers = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(end, attribute_name);
	auto end_to_start = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(end, adjacency_attribute_name);

	for (auto& vertex : end.vert)
	{
		if ( !vertex.IsD() )
		{
			int start_index = end_to_start[vertex];
			if ( start_index >= 0 && start_index < start.vert.size() )
				end_numbers[vertex] = start_numbers[start_index];
		}
	}
}





template<typename MESH>
std::vector<typename BranchTagger<MESH>::Color> BranchTagger<MESH>::generateColorsFromAttribute(MESH const& mesh, std::string const& attribute_name)
{
	auto attribute = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(mesh, attribute_name);

	int min = std::numeric_limits<int>::max();
	int max = std::numeric_limits<int>::min();
	for (int i = 0; i < mesh.VN(); i++)
	{
		if (attribute[i] > max)
			max = attribute[i];
		if (attribute[i] < min)
			min = attribute[i];
	}

	int  range  = max - min + 1;
	auto colors = std::vector<Color>(range + min);
	for (uint i = 0; i < range; i++)
	{
		colors[i + min] = Color::Scatter(range, i, 0.6f, 1.f);
	}

	return colors;
}





template<typename MESH>
void BranchTagger<MESH>::colorizeByAttribute(MESH& mesh, std::vector<Color> const& colors, std::string const& attribute_name)
{
	auto attribute = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(mesh, attribute_name);

	for (int i = 0; i < mesh.vert.size(); i++)
	{
		auto& vertex = mesh.vert[i];
		if ( !vertex.IsD() )
		{
			int value = attribute[i];
			if (value >= 0 && value < colors.size())
				vertex.C() = colors[value];
			else
				vertex.C() = vcg::Color4b(vcg::Color4b::ColorConstant::DarkGray);
		}
	}
}

}

#endif // FILTERCURVATURESKELETON_BRANCH_TAGGER_IMP
