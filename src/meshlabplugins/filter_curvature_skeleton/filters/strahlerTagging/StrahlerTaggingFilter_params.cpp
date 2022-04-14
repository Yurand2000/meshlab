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

#include "StrahlerTaggingFilter.h"

// displayed strings
#define MESH_CATEGORY ""
#define ORIGINAL_MESH_DISPLAYNAME "Original Mesh"
#define ORIGINAL_MESH_DESCRIPTION ""
#define SKELETON_MESH_DISPLAYNAME "Skeleton Mesh"
#define SKELETON_MESH_DESCRIPTION ""
#define TREE_MESH_DISPLAYNAME "Tree Mesh"
#define TREE_MESH_DESCRIPTION ""
#define PARAMETER_CATEGORY "Parameters"
#define STRAHLER_TO_QUALITY_DISPLAYNAME "Save Strahler number to quality"
#define STRAHLER_TO_QUALITY_DESCRIPTION ""

namespace curvatureSkeleton
{

static uint getSelectedMeshIndex(MeshDocument const&);

RichParameterList StrahlerTaggingFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;

	parlst.addParam(RichMesh(PARAM_ORIGINAL_MESH, 0, &m, ORIGINAL_MESH_DISPLAYNAME, ORIGINAL_MESH_DESCRIPTION, false, MESH_CATEGORY));
	parlst.addParam(RichMesh(PARAM_SKELETON_MESH, 0, &m, SKELETON_MESH_DISPLAYNAME, SKELETON_MESH_DESCRIPTION, false, MESH_CATEGORY));
	parlst.addParam(RichMesh(PARAM_TREE_MESH, getSelectedMeshIndex(m), &m, TREE_MESH_DISPLAYNAME, TREE_MESH_DESCRIPTION, false, MESH_CATEGORY));
	parlst.addParam(RichBool(PARAM_STRAHLER_NUMBERS_TO_QUALITY, true, STRAHLER_TO_QUALITY_DISPLAYNAME, STRAHLER_TO_QUALITY_DESCRIPTION, false, PARAMETER_CATEGORY));
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
