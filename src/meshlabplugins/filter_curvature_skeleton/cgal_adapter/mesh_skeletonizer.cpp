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

#define _USE_MATH_DEFINES
#include <cmath>

// default values
#define DELTA_AREA_DEFAULT 0.0001
#define MAX_TRIANGLE_ANGLE_DEFAULT 110
#define QUALITY_TRADEOFF_DEFAULT 0.1
#define MEDIALLY_CENTERING_TRADEOFF_DEFAULT 0.2

namespace CGalAdapter
{
void temporaryCgalMeshCheck(CGALMesh const&); // temp

MeshSkeletonizer::MeshSkeletonizer(CMeshO const& input)
	: skeletonizer(nullptr), delta_area_threshold(DELTA_AREA_DEFAULT), original_area(0), last_area(0), skeleton()
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
		skeletonizer->set_max_triangle_angle((params.max_triangle_angle * M_PI) / 180.0);
	if (params.min_edge_length > 0)
		skeletonizer->set_min_edge_length(params.min_edge_length);
	if (params.quality_speed_tradeoff > 0)
		skeletonizer->set_quality_speed_tradeoff(params.quality_speed_tradeoff);

	if (params.medially_centering_speed_tradeoff > 0)
	{
		skeletonizer->set_is_medially_centered(true);
		skeletonizer->set_medially_centered_speed_tradeoff(params.medially_centering_speed_tradeoff);
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
	generateSkeleton();
	return MeshConverter::convertCGALSkeletonToCMesh(skeleton);
}

MeshSkeletonizer::MeshToSkeletonVertices MeshSkeletonizer::getSkeletonVertexAssociations()
{
	generateSkeleton();
	MeshToSkeletonVertices mesh_to_skeleton = {};
	for (uint i = 0; i < skeleton.m_vertices.size(); i++)
	{
		for(auto orig_vertex : skeleton.m_vertices[i].m_property.vertices)
		{
			mesh_to_skeleton.insert(
				std::make_pair(
					static_cast<uint>(orig_vertex), i
			));
		}
	}
	return mesh_to_skeleton;
}

void MeshSkeletonizer::generateSkeleton()
{
	if (skeleton.m_vertices.empty()) {
		skeletonizer->convert_to_skeleton(skeleton);
	}
}



MeshSkeletonizerParameters::MeshSkeletonizerParameters() :
	max_triangle_angle(-1),
	min_edge_length(-1),
	quality_speed_tradeoff(-1),
	medially_centering_speed_tradeoff(-1),
	delta_area_threshold(-1)
{ }

double calculateMinEdgeLength(CMeshO const&);

MeshSkeletonizerParameters::MeshSkeletonizerParameters(CMeshO const& mesh) :
	max_triangle_angle(MAX_TRIANGLE_ANGLE_DEFAULT),
		min_edge_length(calculateMinEdgeLength(mesh)),
	quality_speed_tradeoff(QUALITY_TRADEOFF_DEFAULT),
	medially_centering_speed_tradeoff(MEDIALLY_CENTERING_TRADEOFF_DEFAULT),
	delta_area_threshold(DELTA_AREA_DEFAULT)
{ }

double calculateMinEdgeLength(CMeshO const& mesh)
{
	auto boundingBox = mesh.trBB();
	return 0.002 * boundingBox.Diag();
}

}
