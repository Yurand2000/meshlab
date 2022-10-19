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

#ifndef FILTERCURVATURESKELETON_EDGE_MESH_UTILS_IMP
#define FILTERCURVATURESKELETON_EDGE_MESH_UTILS_IMP

#include <vector>
#include <queue>
#include <vcg/complex/complex.h>
#include <common/ml_document/cmesh.h>
#include <common/mlexception.h>

namespace curvatureSkeleton
{
	namespace detail
	{

	}

	template<typename MESH>
	std::vector<typename MESH::VertexPointer> EdgeMeshUtils<MESH>::getVerticesInBetween(
		MESH& mesh,
		typename MESH::VertexPointer start_vertex,
		typename MESH::VertexPointer end_vertex
	) {
		std::vector<typename MESH::VertexPointer> path = getVerticesPath(mesh, start_vertex, end_vertex);
		std::queue<typename MESH::VertexPointer> frontier;

		//fill the frontier for the flooding operation
		vcg::tri::UnMarkAll(mesh);
		vcg::tri::Mark(mesh, path.front());
		vcg::tri::Mark(mesh, path.back());
		for (size_t i = 1; i < path.size() - 1; i++) {
			frontier.push(path[i]);
			vcg::tri::Mark(mesh, path[i]);
		}

		//add vertices recursively by flooding
		std::vector<typename MESH::VertexPointer> star;
		do {
			auto* node = frontier.front(); frontier.pop();
			vcg::tri::Mark(mesh, node);

			vcg::edge::VVStarVE(node, star);
			for (auto* adj : star) {
				if (!vcg::tri::IsMarked(mesh, adj))
				{
					path.push_back(adj);
					frontier.push(adj);
				}
			}
		}
		while ( !frontier.empty() );

		return path;
	}

	template<typename MESH>
	std::vector<typename MESH::VertexPointer> EdgeMeshUtils<MESH>::getVerticesPath(
		MESH& mesh,
		typename MESH::VertexPointer start_vertex,
		typename MESH::VertexPointer end_vertex
	) {
		using StarVector = std::vector<typename MESH::VertexPointer>;

		//setup
		vcg::tri::UnMarkAll(mesh);
		std::stack<typename MESH::VertexPointer> parent;
		std::stack<StarVector>                   last_pos;

		parent.push(start_vertex);
		last_pos.emplace();
		vcg::edge::VVStarVE(start_vertex, last_pos.top());

		//explore tree to find the path
		do
		{
			auto* current = parent.top();
			vcg::tri::Mark(mesh, current);
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
				if (!vcg::tri::IsMarked(mesh, next))
				{
					parent.push(next);
					last_pos.emplace();
					vcg::edge::VVStarVE(next, last_pos.top());
				}
			}
		} while (!parent.empty() && parent.top() != end_vertex);

		if (parent.empty())
			throw MLException("Mesh is not connected!");

		//copy path to vector
		std::vector<typename MESH::VertexType*> path;
		path.reserve(parent.size());
		do
		{
			path.push_back( parent.top() );
			parent.pop();
		} while (!parent.empty());
		return path;
	}
}

#endif // FILTERCURVATURESKELETON_EDGE_MESH_UTILS_IMP
