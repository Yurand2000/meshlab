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

#ifndef FILTERCURVATURESKELETON_UTILITY_FUNCTIONS_IMPLEMENTATION
#define FILTERCURVATURESKELETON_UTILITY_FUNCTIONS_IMPLEMENTATION

namespace curvatureSkeleton
{
    template <typename MESH>
    int Utils<MESH>::getVertexIndexInMesh(vcg::Point3<Scalarm> point, MESH const& mesh)
    {
		Scalarm min_sqr_dist = std::numeric_limits<Scalarm>::max();
		int index = 0;
		for (auto& vertex : mesh.vert)
		{
			auto sqr_dist = (point - vertex.cP()).SquaredNorm();
			if (sqr_dist < min_sqr_dist)
			{
				min_sqr_dist = sqr_dist;
				index = vertex.Index();
			}
		}
		return index;
    }

	template <typename MESH>
	typename MESH::VertexType const* Utils<MESH>::getVertexInMesh(vcg::Point3<Scalarm> point, MESH const& mesh)
	{
		Scalarm min_sqr_dist = std::numeric_limits<Scalarm>::max();
		typename MESH::VertexType const* index = &mesh.vert[0];
		for (auto& vertex : mesh.vert)
		{
			auto sqr_dist = (point - vertex.cP()).SquaredNorm();
			if (sqr_dist < min_sqr_dist)
			{
				min_sqr_dist = sqr_dist;
				index = &vertex;
			}
		}
		return index;
	}

	template <typename MESH>
	typename MESH::VertexType* Utils<MESH>::getVertexInMesh(vcg::Point3<Scalarm> point, MESH& mesh)
	{
		Scalarm min_sqr_dist = std::numeric_limits<Scalarm>::max();
		typename MESH::VertexType* index = &mesh.vert[0];
		for (auto& vertex : mesh.vert)
		{
			auto sqr_dist = (point - vertex.cP()).SquaredNorm();
			if (sqr_dist < min_sqr_dist)
			{
				min_sqr_dist = sqr_dist;
				index = &vertex;
			}
		}
		return index;
	}
}

#endif // FILTERCURVATURESKELETON_UTILITY_FUNCTIONS_IMPLEMENTATION
