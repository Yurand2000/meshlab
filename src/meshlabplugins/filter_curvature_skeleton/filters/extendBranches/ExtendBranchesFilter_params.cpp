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

#include "ExtendBranchesFilter.h"

// displayed strings
#define PARAMETER_CATEGORY "Parameters"
#define CONE_ANGLE_DISPLAYNAME "Cone Angle"
#define CONE_ANGLE_DESCRIPTION ""
#define MESH_CATEGORY "Meshes"
#define SKELETON_MESH_DISPLAYNAME "Skeleton Mesh"
#define SKELETON_MESH_DESCRIPTION "The generated skeleton mesh you want its branches to be extended. (Defaults to the selected mesh)"
#define ORIGINAL_MESH_DISPLAYNAME "Original Mesh"
#define ORIGINAL_MESH_DESCRIPTION "The mesh that has generated the skeleton."

namespace curvatureSkeleton
{

static uint getSelectedMeshIndex(MeshDocument const&);

RichParameterList ExtendBranchesFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;
	
	parlst.addParam(RichDynamicFloat(PARAM_CONE_ANGLE, 10, 0, 90, CONE_ANGLE_DISPLAYNAME, CONE_ANGLE_DESCRIPTION, false, PARAMETER_CATEGORY));
	parlst.addParam(RichMesh(PARAM_SKELETON_MESH, getSelectedMeshIndex(m), &m, SKELETON_MESH_DISPLAYNAME, SKELETON_MESH_DESCRIPTION, false, MESH_CATEGORY));
	parlst.addParam(RichMesh(PARAM_ORIGINAL_MESH, 0, &m, ORIGINAL_MESH_DISPLAYNAME, ORIGINAL_MESH_DESCRIPTION, false, MESH_CATEGORY));
	return parlst;
}

uint getSelectedMeshIndex(MeshDocument const& m)
{
	auto selected = m.mm();
	for (uint i = 0; i < m.meshNumber(); i++)
	{
		if (m.getMesh(i) == selected)
			return i;
	}
	return 0;
}

}

