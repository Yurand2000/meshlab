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

namespace CGalAdapter
{

void set_skeletonizer_parameters(CGALSkeletonizer&, MeshSkeletonizerParameters const&);

MeshSkeletonizer::MeshSkeletonizer(CMeshO const& input)
	: skeletonizer(nullptr)
{
	auto cgal_mesh = MeshConverter::convertCMeshToCGALMesh(input);
	skeletonizer   = new CGALSkeletonizer(cgal_mesh);
}

MeshSkeletonizer::MeshSkeletonizer(CMeshO const& input, MeshSkeletonizerParameters const& params)
	: MeshSkeletonizer(input)
{
	set_skeletonizer_parameters(*skeletonizer, params);
}

void set_skeletonizer_parameters(CGALSkeletonizer& skeletonizer, MeshSkeletonizerParameters const& params)
{
	if (params.max_triangle_angle > 0)
		skeletonizer.set_max_triangle_angle(params.max_triangle_angle);
	if (params.min_edge_length > 0)
		skeletonizer.set_min_edge_length(params.min_edge_length);
	if (params.quality_speed_tradeoff > 0)
		skeletonizer.set_quality_speed_tradeoff(params.quality_speed_tradeoff);

	if (params.medially_centered_speed_tradeoff > 0)
	{
		skeletonizer.set_is_medially_centered(true);
		skeletonizer.set_medially_centered_speed_tradeoff(params.medially_centered_speed_tradeoff);
	}
	else
	{
		skeletonizer.set_is_medially_centered(false);
	}
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
	//TODO
	return false;
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
	medially_centered_speed_tradeoff(-1)
{ }

MeshSkeletonizerParameters::MeshSkeletonizerParameters(
	double max_triangle_angle,   double min_edge_length,
	double quality_speed_tradeoff, double medially_centered_speed_tradeoff)
	  : max_triangle_angle(max_triangle_angle), min_edge_length(min_edge_length),
	    quality_speed_tradeoff(quality_speed_tradeoff), medially_centered_speed_tradeoff(medially_centered_speed_tradeoff)
{ }

}
