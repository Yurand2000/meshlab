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

#include "SkeletonMesh.h"
#include <vector>
#include <queue>
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/edge_collapse.h>

namespace curvatureSkeleton
{

typedef std::pair<SkeletonEdge*, int>          EdgeToCollapse;
typedef vcg::tri::UpdateTopology<SkeletonMesh> MeshTopology;
typedef vcg::tri::Allocator<SkeletonMesh>      Allocator;

bool SimplifySkeleton::isMeshConnected(SkeletonMesh const& skeleton)
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

void SimplifySkeleton::collapseTwoConnectedVertices(SkeletonMesh& skeleton, int root_node)
{
	for (auto& vertex : skeleton.vert)
	{
		if (!vertex.IsD() && vertex.Index() != root_node)
			vcg::edge::VEEdgeCollapse<SkeletonMesh>(skeleton, &vertex);
	}
}

void SimplifySkeleton::collapseShortEdges(SkeletonMesh& skeleton, int root_node, Scalarm min_length)
{
	std::vector<EdgeToCollapse> edges_to_collapse;
	min_length = min_length * min_length;

	std::queue<SkeletonVertex*> frontier;
	vcg::tri::UnMarkAll(skeleton);
	auto root_vertex = &skeleton.vert[root_node];
	frontier.push(root_vertex);
	do
	{
		auto*  vertex  = frontier.front();
		vcg::tri::Mark(skeleton, vertex);
		frontier.pop();

		auto it = vcg::edge::VEIterator<SkeletonEdge>(vertex);
		while (!it.End())
		{
			auto* edge      = it.E();
			auto  root_idx  = (edge->V(0) == vertex) ? 0 : 1;
			auto  child_idx = 1 - root_idx;

			//delete edge if it is too short and the root node is never collapsed on any other vertex
			if ( !vcg::tri::IsMarked(skeleton, edge) && edge->V(child_idx) != root_vertex )
			{
				auto sqr_length = (edge->P(0) - edge->P(1)).SquaredNorm();
				if (sqr_length <= min_length)
					edges_to_collapse.emplace_back(edge, root_idx);

				vcg::tri::Mark(skeleton, edge);
			}

			//visit edge children
			auto* child = edge->V(child_idx);
			if ( !vcg::tri::IsMarked(skeleton, child) )
				frontier.push(child);

			++it;
		}
	}
	while (!frontier.empty());

	for (auto& edge : edges_to_collapse)
	{

		vcg::edge::VEEdgeCollapseNonManifold<SkeletonMesh>(
			skeleton, edge.first, edge.second);
	}
}

}
