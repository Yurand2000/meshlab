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

#ifndef FILTERCURVATURESKELETON_SIMPLIFY_SKELETON_IMPLEMENTATION
#define FILTERCURVATURESKELETON_SIMPLIFY_SKELETON_IMPLEMENTATION

#include <vector>
#include <queue>
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/edge_collapse.h>

namespace curvatureSkeleton
{

namespace detail
{

template <typename MESH>
struct EdgeToCollapse
{
	typedef typename MESH::EdgeType EDGE;

	EDGE*   edge;
	int     vertex_to_collapse_onto; // can only be 0 or 1
	Scalarm edge_lenght_sqr;

	EdgeToCollapse() = default;
	EdgeToCollapse(EDGE* edge, int vertex_to_collapse_onto);

	struct less
	{
		bool operator()(EdgeToCollapse const& lhs, EdgeToCollapse const& rhs) const;
	};
};

template<typename MESH> Scalarm getEdgeLengthSqr(typename MESH::EdgeType const* edge);
template<typename MESH> void collapseTwoConnectedVertex(MESH& skeleton, typename MESH::VertexType& vertex, int root_node);

}

template <typename MESH>
bool SimplifySkeleton<MESH>::isMeshConnected(MESH const& skeleton)
{
	for (auto& vertex : skeleton.vert)
	{
		if ( !vertex.IsD() )
		{
			if ( vcg::edge::VEDegree<typename MESH::EdgeType>(&vertex) == 0 )
				return false;
		}
	}
	return true;
}

template<typename MESH>
void SimplifySkeleton<MESH>::collapseTwoConnectedVertices(MESH& skeleton, int root_node)
{
	for (auto& vertex : skeleton.vert)
	{
		detail::collapseTwoConnectedVertex(skeleton, vertex, root_node);
	}
}

template<typename MESH>
void detail::collapseTwoConnectedVertex(MESH& skeleton, typename MESH::VertexType& vertex, int root_node)
{
	if ( !vertex.IsD() && vertex.Index() != root_node )
		vcg::edge::VEEdgeCollapse<MESH>(skeleton, &vertex);
}

template<typename MESH>
void SimplifySkeleton<MESH>::collapseShortEdges(MESH& skeleton, int root_node, Scalarm min_length)
{
	std::vector<detail::EdgeToCollapse<MESH>> edges_to_collapse;
	min_length = min_length * min_length;

	std::queue<typename MESH::VertexType*> frontier;
	vcg::tri::UnMarkAll(skeleton);
	auto root_vertex = &skeleton.vert[root_node];
	frontier.push(root_vertex);

	do
	{
		auto*  vertex  = frontier.front();
		vcg::tri::Mark(skeleton, vertex);
		frontier.pop();

		auto it = vcg::edge::VEIterator<typename MESH::EdgeType>(vertex);
		while (!it.End())
		{
			auto* edge      = it.E();
			auto  root_idx  = (edge->V(0) == vertex) ? 0 : 1;

			//delete edge if it is too short and the root node is never collapsed on any other vertex
			if ( !vcg::tri::IsMarked(skeleton, edge) && edge->V1(root_idx) != root_vertex )
			{
				auto sqr_length = detail::getEdgeLengthSqr<MESH>(edge); 
				if (sqr_length <= min_length)
					edges_to_collapse.emplace_back(edge, root_idx);

				vcg::tri::Mark(skeleton, edge);
			}

			//visit edge children
			auto* child = edge->V1(root_idx);
			if ( !vcg::tri::IsMarked(skeleton, child) )
				frontier.push(child);

			++it;
		}
	}
	while (!frontier.empty());

	std::sort(edges_to_collapse.begin(), edges_to_collapse.end(), typename detail::EdgeToCollapse<MESH>::less());
	for (auto& edge_data : edges_to_collapse)
	{
		if ( !edge_data.edge->IsD() )
		{
			auto updated_length = detail::getEdgeLengthSqr<MESH>(edge_data.edge);
			if (updated_length <= min_length)
			{
				auto& collapsing_vertex = *edge_data.edge->V0(edge_data.vertex_to_collapse_onto);
				vcg::edge::VEEdgeCollapseNonManifold<MESH>(
					skeleton, edge_data.edge, edge_data.vertex_to_collapse_onto);

				detail::collapseTwoConnectedVertex(skeleton, collapsing_vertex, root_node);
			}
		}
	}
}

template<typename MESH>
detail::EdgeToCollapse<MESH>::EdgeToCollapse(EDGE* edge, int vertex_to_collapse_onto) :
	edge( edge ),
	vertex_to_collapse_onto( vertex_to_collapse_onto ),
	edge_lenght_sqr( detail::getEdgeLengthSqr<MESH>(edge) )
{ }

template<typename MESH>
bool detail::EdgeToCollapse<MESH>::less::operator()(EdgeToCollapse<MESH> const& lhs, EdgeToCollapse<MESH> const& rhs) const
{
	return lhs.edge_lenght_sqr < rhs.edge_lenght_sqr;
}

template <typename MESH>
Scalarm detail::getEdgeLengthSqr(typename MESH::EdgeType const* edge)
{
	return vcg::SquaredDistance(edge->P(0), edge->P(1));
}

} // namespace curvatureSkeleton

#endif // FILTERCURVATURESKELETON_SIMPLIFY_SKELETON_IMPLEMENTATION
