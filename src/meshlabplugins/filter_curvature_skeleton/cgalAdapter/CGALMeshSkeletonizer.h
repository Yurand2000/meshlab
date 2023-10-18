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

#ifndef FILTERCURVATURESKELETON_CGAL_MESH_SKELETONIZER
#define FILTERCURVATURESKELETON_CGAL_MESH_SKELETONIZER

#include <common/ml_document/base_types.h>
#include <CGAL/Mean_curvature_flow_skeletonization.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

namespace curvatureSkeleton
{

class CGALMeshSkeletonizer
{
public:
	struct Parameters;

	typedef CGAL::Simple_cartesian<Scalarm> Kernel;
	typedef Kernel::Point_3                 Point;
	typedef CGAL::Surface_mesh<Point>       Mesh;
	typedef CGAL::SM_Vertex_index           VertexIndex;

	typedef CGAL::Mean_curvature_flow_skeletonization<Mesh> Skeletonizer;
	typedef Skeletonizer::Skeleton                          Skeleton;
	typedef std::unordered_map<int, int> MeshToSkeletonVertices;

public:
	CGALMeshSkeletonizer(Mesh const& input);
	CGALMeshSkeletonizer(Mesh const& input, Parameters const& params);

	CGALMeshSkeletonizer(CGALMeshSkeletonizer& copy) = delete;
	CGALMeshSkeletonizer& operator=(CGALMeshSkeletonizer& copy) = delete;

	void computeStep();
	bool hasConverged();
	Skeleton getSkeleton();
	MeshToSkeletonVertices getSkeletonVertexAssociations();

private:
	Skeletonizer skeletonizer;

	double delta_area_threshold;
	double original_area;
	double last_area;

	Skeleton skeleton;

	void setSkeletonizerParameters(Parameters const& params);
	void generateSkeleton();

public:
	struct Parameters
	{
		/* All parameters are ignored if set to negative values.
		 * They will use the default values from the CGAL library.
		 */

		double max_triangle_angle                = 110;
		double min_edge_length                   = -1;
		double quality_speed_tradeoff            = 20;
		double medially_centering_speed_tradeoff = 20;
		double delta_area_threshold              = 0.0001;

		Parameters() = default;

		template <typename MYMESH>
		Parameters(MYMESH const& mesh)
		{
			min_edge_length = mesh.trBB().Diag() * 0.0005; 
		}

		template <typename MYMESH>
		Parameters(MYMESH const& mesh, double min_edge_scale)
		{
			min_edge_length = mesh.trBB().Diag() * min_edge_scale;
		}
	};
};

}

#endif //FILTERCURVATURESKELETON_CGAL_MESH_SKELETONIZER
