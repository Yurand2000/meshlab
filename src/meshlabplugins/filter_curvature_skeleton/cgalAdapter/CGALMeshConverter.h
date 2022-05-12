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
	typedef CGAL::Mean_curvature_flow_skeletonization<Mesh>::Meso_skeleton MesoSkeleton;



	static Mesh toCGALMesh(MYMESH const& mesh)
	{
		Mesh new_mesh;
		new_mesh.set_recycle_garbage(false);

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

		new_mesh.set_recycle_garbage(true);
		return new_mesh;
	}



	static MYMESH CGALMesoSkeletonToMesh(MesoSkeleton const& meso_skeleton)
	{
		using Allocator = vcg::tri::Allocator<MYMESH>;
		using VertexIndices = std::unordered_map<int, int>;

		MYMESH        new_mesh = {};
		VertexIndices indices  = {};

		//add vertices
		int index = 0;
		for (auto it = meso_skeleton.vertices_begin(); it != meso_skeleton.vertices_end(); it++)
		{
			auto const& vertex = it->point();

			vcg::Point3<Scalarm> new_vertex = { vertex.x(), vertex.y(), vertex.z() };
			Allocator::AddVertex(new_mesh, new_vertex);

			indices.emplace(it->id(), index);
			index++;
		}

		//add edges
		for (auto it = meso_skeleton.edges_begin(); it != meso_skeleton.edges_end(); it++)
		{
			auto index1 = indices.at( it->vertex()->id() );
			auto index2 = indices.at( it->opposite()->vertex()->id() );

			Allocator::AddEdge(new_mesh, index1, index2);
		}

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
