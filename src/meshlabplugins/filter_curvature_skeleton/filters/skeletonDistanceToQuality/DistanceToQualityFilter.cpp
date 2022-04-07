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

#include "DistanceToQualityFilter.h"

#include <vcg/complex/allocate.h>
#include "filter_curvature_skeleton.h"
#include "common/additionalAttributeNames.h"

#define F_FILTERID	  FilterCurvatureSkeleton::SKELETON_DISTANCE_TO_MESH_QUALITY
#define F_DISPLAYNAME "Skeleton Distance to Quality"
#define F_DESCRIPTION "After skeletonizing any mesh, you can set the values of the vertex quality with the distance from the vertex it has collapsed onto the skeleton."
#define F_CATEGORY    FilterPlugin::Other
#define F_PYTHON_NAME "skeleton_distance_to_mesh_quality"
#define F_ARITY       FilterPlugin::FilterArity::SINGLE_MESH
#define F_PRECONDS    MeshModel::MM_NONE
#define F_POSTCONDS   MeshModel::MM_VERTQUALITY

namespace curvatureSkeleton
{

SkeletonDistanceToMeshQualityFilter::SkeletonDistanceToMeshQualityFilter() :
	TemplateFilter(F_FILTERID, F_DISPLAYNAME, F_DESCRIPTION, F_CATEGORY,
		F_PYTHON_NAME, F_ARITY, F_PRECONDS, F_POSTCONDS) { }

std::map<std::string, QVariant> SkeletonDistanceToMeshQualityFilter::applyFilter(
	FilterPlugin const& plugin,
	RichParameterList const&,
	MeshDocument& document,
	unsigned int&,
	vcg::CallBackPos* callback)
{
	auto    meshModel = document.mm();
	CMeshO& mesh      = meshModel->cm;

	auto iterator = vcg::tri::Allocator<CMeshO>::FindPerVertexAttribute<Scalarm>(
		mesh, ATTRIBUTE_MESH_TO_SKELETON_DISTANCE_NAME);

	if(vcg::tri::Allocator<CMeshO>::IsValidHandle(mesh, iterator))
	{
		meshModel->setMeshModified(true);
		meshModel->updateDataMask(MeshModel::MM_VERTQUALITY);
		for (uint i = 0; i < mesh.vert.size(); i++)
		{
			auto& vert = mesh.vert[i];
			vert.Q()   = iterator[i];
		}
	}	
	else
	{
		throw MLException("The selected mesh has no attribute by name \"" ATTRIBUTE_MESH_TO_SKELETON_DISTANCE_NAME "\"."\
			" Have you skeletonized the mesh first?");
	}

	return std::map<std::string, QVariant>();
}

}
