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

#include "mesh_skeletonizer.h"
#include "mesh_converter.h"
#include <common/mlexception.h> //temp

#define DELTA_AREA_THRESHOLD 0.0001

namespace CGalAdapter
{
void temporaryCgalMeshCheck(CGALMesh const&); // temp

MeshSkeletonizer::MeshSkeletonizer(CMeshO const& input)
	: skeletonizer(nullptr), delta_area_threshold(DELTA_AREA_THRESHOLD), original_area(0), last_area(0)
{
	auto cgal_mesh = MeshConverter::convertCMeshToCGALMesh(input);
	temporaryCgalMeshCheck(cgal_mesh);
	skeletonizer   = new CGALSkeletonizer(cgal_mesh);
	original_area  = CGAL::Polygon_mesh_processing::area(skeletonizer->meso_skeleton());
	last_area      = original_area;
}

void temporaryCgalMeshCheck(CGALMesh const& mesh) // temp
{
	if(!CGAL::is_closed(mesh))
	{
		throw MLException("Given mesh is not closed.");
	}
}

MeshSkeletonizer::MeshSkeletonizer(CMeshO const& input, MeshSkeletonizerParameters const& params)
	: MeshSkeletonizer(input)
{
	set_skeletonizer_parameters(params);
}

void MeshSkeletonizer::set_skeletonizer_parameters(MeshSkeletonizerParameters const& params)
{
	if (params.max_triangle_angle > 0)
		skeletonizer->set_max_triangle_angle(params.max_triangle_angle);
	if (params.min_edge_length > 0)
		skeletonizer->set_min_edge_length(params.min_edge_length);
	if (params.quality_speed_tradeoff > 0)
		skeletonizer->set_quality_speed_tradeoff(params.quality_speed_tradeoff);

	if (params.medially_centered_speed_tradeoff > 0)
	{
		skeletonizer->set_is_medially_centered(true);
		skeletonizer->set_medially_centered_speed_tradeoff(params.medially_centered_speed_tradeoff);
	}
	else
	{
		skeletonizer->set_is_medially_centered(false);
	}

	if (params.delta_area_threshold > 0)
		delta_area_threshold = params.delta_area_threshold;
}

MeshSkeletonizer::~MeshSkeletonizer()
{
	if (skeletonizer != nullptr)
    {
		delete skeletonizer;
    }
}

void MeshSkeletonizer::computeStep()
{
	skeletonizer->contract();
}

bool MeshSkeletonizer::hasConverged()
{
	double area       = CGAL::Polygon_mesh_processing::area( skeletonizer->meso_skeleton() );
	double area_ratio = fabs(last_area - area) / original_area;
	last_area         = area;
	return area_ratio < delta_area_threshold;
}

CMeshO MeshSkeletonizer::getMesoSkeleton()
{
	CGALMesoSkeleton meso_skeleton = skeletonizer->meso_skeleton();
	return MeshConverter::convertCGALMesoSkeletonToCMesh(meso_skeleton);
}

CMeshO MeshSkeletonizer::getSkeleton()
{
	CGALSkeleton skeleton = {};
	skeletonizer->convert_to_skeleton(skeleton);
	return MeshConverter::convertCGALSkeletonToCMesh(skeleton);
}



MeshSkeletonizerParameters::MeshSkeletonizerParameters() :
	max_triangle_angle(-1),
	min_edge_length(-1),
	quality_speed_tradeoff(-1),
	medially_centered_speed_tradeoff(-1),
	delta_area_threshold(-1)
{ }

MeshSkeletonizerParameters::MeshSkeletonizerParameters(
	double max_triangle_angle, double min_edge_length,
	double quality_speed_tradeoff, double medially_centered_speed_tradeoff,
	double zero_threshold)
	  : max_triangle_angle(max_triangle_angle), min_edge_length(min_edge_length),
	    quality_speed_tradeoff(quality_speed_tradeoff), medially_centered_speed_tradeoff(medially_centered_speed_tradeoff),
		delta_area_threshold(delta_area_threshold)
{ }

}
