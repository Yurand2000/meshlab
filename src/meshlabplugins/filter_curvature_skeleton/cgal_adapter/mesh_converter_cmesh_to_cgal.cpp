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

#include <unordered_map>

namespace CGalAdapter
{

typedef std::unordered_map<CVertexO const*, CGALVertexIndex> CMeshVertexIndices;

void addVertices(CGALMesh&, CMeshVertexIndices&, CMeshO const&);
void addEdges(CGALMesh&, CMeshVertexIndices const&, CMeshO const&);
void addFaces(CGALMesh&, CMeshVertexIndices const&, CMeshO const&);

CGALMesh MeshConverter::convertCMeshToCGALMesh(CMeshO const& mesh)
{
	CGALMesh           new_mesh       = {};
	CMeshVertexIndices vertex_indices = {};

	addVertices(new_mesh, vertex_indices, mesh);
	addEdges(new_mesh, vertex_indices, mesh);
	addFaces(new_mesh, vertex_indices, mesh);

	return new_mesh;
}

void addVertices(CGALMesh& new_mesh, CMeshVertexIndices& vertex_indices, CMeshO const& mesh)
{
	for (int i = 0; i < mesh.vert.size(); i++) {
		CVertexO const* vertex      = &mesh.vert[i];
		CGALPoint       cgal_vertex = CGALPoint(vertex->P().X(), vertex->P().Y(), vertex->P().Z());

		auto cgal_vertex_index = new_mesh.add_vertex(cgal_vertex);
		vertex_indices.insert(std::make_pair(vertex, cgal_vertex_index));
	}
}

void addEdges(CGALMesh& new_mesh, CMeshVertexIndices const& vertex_indices, CMeshO const& mesh)
{
	for (int i = 0; i < mesh.edge.size(); i++) {
		CEdgeO const& edge   = mesh.edge[i];
		auto          index1 = vertex_indices.at(edge.V(0));
		auto          index2 = vertex_indices.at(edge.V(1));

		new_mesh.add_edge(index1, index2);
	}
}

void addFaces(CGALMesh& new_mesh, CMeshVertexIndices const& vertex_indices, CMeshO const& mesh)
{
	for (int i = 0; i < mesh.face.size(); i++) {
		CFaceO const& face   = mesh.face[i];
		auto          index1 = vertex_indices.at(face.V(0));
		auto          index2 = vertex_indices.at(face.V(1));
		auto          index3 = vertex_indices.at(face.V(2));

		new_mesh.add_face(index1, index2, index3);
	}
}

}
