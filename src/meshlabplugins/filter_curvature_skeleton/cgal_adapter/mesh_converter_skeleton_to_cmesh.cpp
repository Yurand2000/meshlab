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

#include "mesh_converter.h"
#include <vcg/complex/algorithms/polygon_support.h>

#include <unordered_map>

namespace CGalAdapter
{

typedef vcg::tri::Allocator<CMeshO> Allocator;

void addSkeletonVertices(CMeshO&, CGALSkeleton const&);
void addSkeletonEdges(CMeshO&, CGALSkeleton const&);

CMeshO MeshConverter::convertCGALSkeletonToCMesh(CGALSkeleton const& skeleton)
{
	CMeshO new_skeleton = {};
	addSkeletonVertices(new_skeleton, skeleton);
	addSkeletonEdges(new_skeleton, skeleton);

	return new_skeleton;
}

void addSkeletonVertices(CMeshO& mesh, CGALSkeleton const& skeleton)
{
	for (int i = 0; i < skeleton.m_vertices.size(); i++)
	{
		auto const& vertex = skeleton.m_vertices[i].m_property.point;

		vcg::Point3d new_vertex = { vertex.x(), vertex.y(), vertex.z() };
		Allocator::AddVertex(mesh, new_vertex);
	}
}

void addSkeletonEdges(CMeshO& mesh, CGALSkeleton const& skeleton)
{
	for (auto const& edge : skeleton.m_edges)
	{
		Allocator::AddEdge(mesh, edge.m_source, edge.m_target);
	}
}

}
