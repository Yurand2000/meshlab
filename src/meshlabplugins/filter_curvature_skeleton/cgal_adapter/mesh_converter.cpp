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
    CGALMesh CGalMeshConverter::convertCMeshToCGALPolyhedron(CMeshO const& mesh)
    {
	    CGALMesh new_mesh = {};

        std::unordered_map<CVertexO const*, CGALVertexIndex> vertex_indices = {};

        for (int i = 0; i < mesh.vert.size(); i++) {
			CVertexO const* vertex = &mesh.vert[i];
			auto new_vertex = new CGALPoint(
                vertex->P().X(), vertex->P().Y(), vertex->P().Z());

            auto new_vertex_index = new_mesh.add_vertex(*new_vertex);
			vertex_indices.insert(std::make_pair(vertex, new_vertex_index));
        }

        for (int i = 0; i < mesh.edge.size(); i++)
        {
			CEdgeO const& edge = mesh.edge[i];
			auto        index1 = vertex_indices.at(edge.V(0));
			auto        index2 = vertex_indices.at(edge.V(1));

			new_mesh.add_edge(index1, index2);
        }

        for (int i = 0; i < mesh.face.size(); i++)
        {
			CFaceO const& face = mesh.face[i];
			auto          index1 = vertex_indices.at(face.V(0));
			auto          index2 = vertex_indices.at(face.V(1));
			auto          index3 = vertex_indices.at(face.V(2));

            new_mesh.add_face(index1, index2, index3);
        }

        return new_mesh;
	}

	CMeshO CGalMeshConverter::convertCGALSkeletontoCMesh(CGALSkeleton const& skeleton)
	{
		auto allocator = vcg::tri::Allocator<CMeshO>();

		CMeshO new_skeleton = {};
		for (int i = 0; i < skeleton.m_vertices.size(); i++)
		{
			auto const& vertex = skeleton.m_vertices[i].m_property.point;

			vcg::Point3d new_vertex = { vertex.x(), vertex.y(), vertex.z() };
			allocator.AddVertex(new_skeleton, new_vertex);
		}

		for (auto const& edge : skeleton.m_edges)
		{

			allocator.AddEdge(new_skeleton, edge.m_source, edge.m_target);
		}

		return new_skeleton;
	}
}
