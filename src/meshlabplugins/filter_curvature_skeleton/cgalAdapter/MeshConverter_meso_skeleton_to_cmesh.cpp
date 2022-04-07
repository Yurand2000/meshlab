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

#include "MeshConverter.h"
#include <vcg/complex/algorithms/polygon_support.h>

#include <unordered_map>

namespace curvatureSkeleton { namespace CGalAdapter
{

typedef vcg::tri::Allocator<CMeshO>        Allocator;
typedef std::unordered_map<size_t, size_t> VertexIndices;

static void addVertices(CMeshO&, VertexIndices&, CGALMesoSkeleton const&);
static void addEdges(CMeshO&, VertexIndices const&, CGALMesoSkeleton const&);
static void addFaces(CMeshO&, VertexIndices const&, CGALMesoSkeleton const&);

CMeshO convertCGALMesoSkeletonToCMesh(CGALMesoSkeleton const& meso_skeleton)
{
	CMeshO new_mesh = {};
	VertexIndices indices = {};

	addVertices(new_mesh, indices, meso_skeleton);
	addEdges(new_mesh, indices, meso_skeleton);
	addFaces(new_mesh, indices, meso_skeleton);

	return new_mesh;
}

void addVertices(CMeshO& new_mesh, VertexIndices& indices, CGALMesoSkeleton const& meso_skeleton)
{
	size_t index = 0;
	for(auto it = meso_skeleton.vertices_begin(); it != meso_skeleton.vertices_end(); it++)
	{
		auto const& vertex = it->point();

		vcg::Point3d new_vertex = { vertex.x(), vertex.y(), vertex.z() };
		Allocator::AddVertex(new_mesh, new_vertex);
		indices.insert(std::make_pair(it->id(), index));
		index++;
	}
}

void addEdges(CMeshO& new_mesh, VertexIndices const& indices, CGALMesoSkeleton const& meso_skeleton)
{
	for (auto it = meso_skeleton.edges_begin(); it != meso_skeleton.edges_end(); it++)
	{
		auto index1 = indices.at(it->vertex()->id());
		auto index2 = indices.at(it->opposite()->vertex()->id());

		Allocator::AddEdge(new_mesh, index1, index2);
	}
}

void addFaces(CMeshO& new_mesh, VertexIndices const& indices, CGALMesoSkeleton const& meso_skeleton)
{
	for (auto it = meso_skeleton.facets_begin(); it != meso_skeleton.facets_end(); it++)
	{
		if (it->is_triangle())
		{
			auto vertex_it = it->facet_begin();
			auto index1    = indices.at(vertex_it->vertex()->id()); vertex_it++;
			auto index2    = indices.at(vertex_it->vertex()->id()); vertex_it++;
			auto index3    = indices.at(vertex_it->vertex()->id());

			Allocator::AddFace(new_mesh, index1, index2, index3);
		}
		else
		{
			throw std::runtime_error("non triangle face needs to be splitted, not implemented!");
		}
	}
}

} }
