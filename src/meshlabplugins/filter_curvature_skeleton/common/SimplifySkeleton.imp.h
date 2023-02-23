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
template<typename MESH> void collapseShortLeafs(MESH& skeleton, int root_node, Scalarm min_length);

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
	//collapse leafs with a fix-point algorithm. Tries to collapse leafs shorter than the given length until no more leafs are collapsed.
	auto last_edge_count = 0;
	auto edge_count = skeleton.EN();
	do
	{
		last_edge_count = edge_count;
		detail::collapseShortLeafs(skeleton, root_node, min_length);
		vcg::tri::UpdateTopology<MESH>::VertexEdge(skeleton);
		edge_count = skeleton.EN();
	} while (last_edge_count != edge_count);
}

template<typename MESH>
void detail::collapseShortLeafs(MESH& skeleton, int root_node, Scalarm min_length)
{
	std::vector<detail::EdgeToCollapse<MESH>> edges_to_collapse;
	auto min_length_sqr = min_length * min_length;

	for (auto& edge : skeleton.edge)
	{
		auto sqr_length = detail::getEdgeLengthSqr<MESH>(&edge);
		if (sqr_length > min_length_sqr)
			continue;

		auto v0_adj_num = vcg::edge::VEDegree<typename MESH::EdgeType>( edge.V(0) );
		auto v1_adj_num = vcg::edge::VEDegree<typename MESH::EdgeType>( edge.V(1) );

		if (v0_adj_num == 1) {
			edges_to_collapse.emplace_back(&edge, 1);
		}
		else if (v1_adj_num == 1) {
			edges_to_collapse.emplace_back(&edge, 0);
		}
	}

	std::sort(edges_to_collapse.begin(), edges_to_collapse.end(), typename detail::EdgeToCollapse<MESH>::less());
	for (auto& edge_data : edges_to_collapse)
	{
		if ( !edge_data.edge->IsD() )
		{
			auto updated_length = detail::getEdgeLengthSqr<MESH>(edge_data.edge);
			if (updated_length <= min_length_sqr)
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
