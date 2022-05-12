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

#include "CGALMeshSkeletonizer.h"

#define _USE_MATH_DEFINES
#include <cmath>

#define DELTA_AREA_DEFAULT 0.0001

namespace curvatureSkeleton
{

CGALMeshSkeletonizer::CGALMeshSkeletonizer(Mesh const& input)
	: skeletonizer(input), delta_area_threshold(DELTA_AREA_DEFAULT), original_area(0), last_area(0), skeleton()
{
	original_area  = CGAL::Polygon_mesh_processing::area(skeletonizer.meso_skeleton());
	last_area      = original_area;
}

CGALMeshSkeletonizer::CGALMeshSkeletonizer(Mesh const& input, Parameters const& params)
	: CGALMeshSkeletonizer(input)
{
	setSkeletonizerParameters(params);
}

void CGALMeshSkeletonizer::setSkeletonizerParameters(Parameters const& params)
{
	if (params.max_triangle_angle > 0)
		skeletonizer.set_max_triangle_angle((params.max_triangle_angle * Scalarm(M_PI)) / Scalarm(180.0));
	if (params.min_edge_length > 0)
		skeletonizer.set_min_edge_length(params.min_edge_length);
	if (params.quality_speed_tradeoff > 0)
		skeletonizer.set_quality_speed_tradeoff(Scalarm(2.0) / params.quality_speed_tradeoff);

	if (params.medially_centering_speed_tradeoff > 0)
	{
		skeletonizer.set_is_medially_centered(true);
		skeletonizer.set_medially_centered_speed_tradeoff(Scalarm(2.0) / params.medially_centering_speed_tradeoff);
	}
	else
	{
		skeletonizer.set_is_medially_centered(false);
	}

	if (params.delta_area_threshold > 0)
		delta_area_threshold = params.delta_area_threshold;
}

void CGALMeshSkeletonizer::computeStep()
{
	skeletonizer.contract();
}

bool CGALMeshSkeletonizer::hasConverged()
{
	double area       = CGAL::Polygon_mesh_processing::area( skeletonizer.meso_skeleton() );
	double area_ratio = fabs(last_area - area) / original_area;
	last_area         = area;
	return area_ratio < delta_area_threshold;
}

CGALMeshSkeletonizer::MesoSkeleton CGALMeshSkeletonizer::getMesoSkeleton()
{
	return skeletonizer.meso_skeleton();
}

CGALMeshSkeletonizer::Skeleton CGALMeshSkeletonizer::getSkeleton()
{
	generateSkeleton();
	return skeleton;
}

CGALMeshSkeletonizer::MeshToSkeletonVertices CGALMeshSkeletonizer::getSkeletonVertexAssociations()
{
	generateSkeleton();
	MeshToSkeletonVertices mesh_to_skeleton;
	for (int i = 0; i < skeleton.m_vertices.size(); i++)
	{
		for (auto const& orig_vertex : skeleton.m_vertices[i].m_property.vertices)
		{
			mesh_to_skeleton.emplace( static_cast<int>( orig_vertex.idx() ), i );
		}
	}
	return mesh_to_skeleton;
}

void CGALMeshSkeletonizer::generateSkeleton()
{
	if ( skeleton.m_vertices.empty() )
	{
		skeletonizer.convert_to_skeleton(skeleton);
	}
}

}
