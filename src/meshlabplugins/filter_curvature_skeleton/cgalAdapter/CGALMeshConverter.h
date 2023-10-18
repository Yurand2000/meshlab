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

#ifndef FILTERCURVATURESKELETON_CGAL_MESH_CONVERTER
#define FILTERCURVATURESKELETON_CGAL_MESH_CONVERTER

#include <unordered_map>
#include <common/ml_document/base_types.h>
#include <CGAL/Mean_curvature_flow_skeletonization.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

namespace curvatureSkeleton
{

template <typename MYMESH>
class CGALMeshConverter
{
public:
	typedef CGAL::Simple_cartesian<Scalarm> Kernel;
	typedef Kernel::Point_3                 Point;
	typedef CGAL::Surface_mesh<Point>       Mesh;
	typedef CGAL::SM_Vertex_index           VertexIndex;

	typedef CGAL::Mean_curvature_flow_skeletonization<Mesh>::Skeleton      Skeleton;

	static Mesh toCGALMesh(MYMESH const& mesh)
	{
		Mesh new_mesh;

		//add vertices
		for (auto& vert : mesh.vert)
		{
			Point cgal_vertex = { vert.P().X(), vert.P().Y(), vert.P().Z() };
			new_mesh.add_vertex(cgal_vertex);
		}

		//add edges
		for (auto& edge : mesh.edge)
		{
			new_mesh.add_edge(
				VertexIndex(edge.V(0)->Index()),
				VertexIndex( edge.V(1)->Index() )
			);
		}

		//add faces
		for (auto& face : mesh.face)
		{
			new_mesh.add_face(
				VertexIndex(face.V(0)->Index()),
				VertexIndex(face.V(1)->Index()),
				VertexIndex(face.V(2)->Index())
			);
		}

		return new_mesh;
	}

	static MYMESH CGALSkeletonToMesh(Skeleton const& skeleton)
	{
		using Allocator = vcg::tri::Allocator<MYMESH>;

		MYMESH mesh = {};

		//add vertices
		for (int i = 0; i < skeleton.m_vertices.size(); i++)
		{
			auto const& vertex = skeleton.m_vertices[i].m_property.point;

			vcg::Point3d new_vertex = { vertex.x(), vertex.y(), vertex.z() };
			Allocator::AddVertex(mesh, new_vertex);
		}

		//add edges
		for (auto const& edge : skeleton.m_edges)
		{
			Allocator::AddEdge(mesh, edge.m_source, edge.m_target);
		}

		return mesh;
	}

private:
	~CGALMeshConverter() = delete;

};

}

#endif //FILTERCURVATURESKELETON_CGAL_MESH_CONVERTER
