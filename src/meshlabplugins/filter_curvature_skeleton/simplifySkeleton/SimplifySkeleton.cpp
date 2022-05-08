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

#include "SimplifySkeleton.h"

#include "common/SkeletonMesh.h"
#include <vector>
#include <functional>
#include <unordered_set>
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/edge_collapse.h>

namespace curvatureSkeleton
{

typedef std::pair<SkeletonEdge*, int>          EdgeToCollapse;
typedef vcg::tri::UpdateTopology<SkeletonMesh> MeshTopology;
typedef vcg::tri::Allocator<SkeletonMesh>      Allocator;

bool isMeshConnected(SkeletonMesh const& skeleton)
{
	for (auto& vertex : skeleton.vert)
	{
		if ( !vertex.IsD() )
		{
			if ( vcg::edge::VEDegree<SkeletonEdge>(&vertex) == 0 )
				return false;
		}
	}
	return true;
}

void collapseTwoConnectedVertices(SkeletonMesh& skeleton)
{
	for (auto& vertex : skeleton.vert)
	{
		if ( !vertex.IsD() )
			vcg::edge::VEEdgeCollapse<SkeletonMesh>(skeleton, &vertex);
	}
}

void collapseShortEdges(SkeletonMesh& skeleton, int root_node, Scalarm min_length)
{
	std::vector<EdgeToCollapse> edge_to_collapse;
	min_length = min_length * min_length;

	std::queue<SkeletonVertex*>         frontier;
	std::unordered_set<SkeletonVertex*> visited;
	std::unordered_set<SkeletonEdge*> visited_edges;
	frontier.push(&skeleton.vert[root_node]);	

	do
	{
		auto*  vertex  = frontier.front();
		visited.insert(vertex);
		frontier.pop();

		auto it = vcg::edge::VEIterator<SkeletonEdge>(vertex);
		while (!it.End())
		{
			auto* edge      = it.E();
			auto  root_idx  = (edge->V(0) == vertex) ? 0 : 1;
			auto  child_idx = 1 - root_idx;

			//delete edge if it is too short
			if (visited_edges.count(edge) == 0)
			{
				auto sqr_length = (edge->P(0) - edge->P(1)).SquaredNorm();
				if (sqr_length <= min_length)
					edge_to_collapse.emplace_back(edge, root_idx);

				visited_edges.insert(edge);
			}

			//visit edge children
			auto* child     = edge->V(child_idx);
			if ( visited.count(child) == 0 )
				frontier.push(child);

			++it;
		}
	}
	while (!frontier.empty());



	for (auto& edge_to_collapse : edge_to_collapse)
	{
		std::vector<SkeletonEdge*> edges;

		auto root_index = edge_to_collapse.second;
		auto child_index = 1 - root_index;
		auto* root_vertex  = edge_to_collapse.first->V(root_index);
		auto* child_vertex = edge_to_collapse.first->V(child_index);
		vcg::edge::VEStarVE(child_vertex, edges);
		edges.erase( std::find(edges.begin(), edges.end(), edge_to_collapse.first) );

		for (auto* edge : edges)
		{
			auto detach_index = (edge->V(0) == child_vertex) ? 0 : 1;
			vcg::edge::VEDetach(*edge, detach_index);
			edge->V(detach_index) = root_vertex;
			vcg::edge::VEAppend(edge, detach_index);
		}

		vcg::edge::VEDetach(*edge_to_collapse.first);
		Allocator::DeleteEdge(skeleton, *edge_to_collapse.first);
		Allocator::DeleteVertex(skeleton, *child_vertex);
	}
}

}
