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

#include "MeasuresComputeFilter.h"

#define MESH_CATEGORY "Meshes"
#define ORIGINAL_MESH_DISPLAYNAME "Original Mesh"
#define ORIGINAL_MESH_DESCRIPTION "The original mesh which has generated the <b>Skeleton</b> mesh"
#define SKELETON_MESH_DISPLAYNAME "Skeleton Mesh"
#define SKELETON_MESH_DESCRIPTION "The skeleton mesh."
#define TREE_MESH_DISPLAYNAME "Tree Mesh"
#define TREE_MESH_DESCRIPTION "The tree mesh."

#define PARAMETER_CATEGORY "Parameters"
#define ATTRIBUTE_DISPLAYNAME "Ordering Attribute"
#define ATTRIBUTE_DESCRIPTION ""
#define ATTRIBUTE_ENUM { "Hack ordering", "Strahler ordering" }

namespace curvatureSkeleton
{

static uint tryGetOriginalMeshIndex(MeshDocument const&);
static uint tryGetSkeletonMeshIndex(MeshDocument const&);
static uint tryGetTreeMeshIndex(MeshDocument const&);

RichParameterList MeasuresComputeFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;

	parlst.addParam(RichMesh(PARAM_ORIGINAL_MESH, tryGetOriginalMeshIndex(m), &m, ORIGINAL_MESH_DISPLAYNAME, ORIGINAL_MESH_DESCRIPTION, false, MESH_CATEGORY));
	parlst.addParam(RichMesh(PARAM_SKELETON_MESH, tryGetSkeletonMeshIndex(m), &m, SKELETON_MESH_DISPLAYNAME, SKELETON_MESH_DESCRIPTION, false, MESH_CATEGORY));
	parlst.addParam(RichMesh(PARAM_TREE_MESH, tryGetTreeMeshIndex(m), &m, TREE_MESH_DISPLAYNAME, TREE_MESH_DESCRIPTION, false, MESH_CATEGORY));
	parlst.addParam(RichEnum(PARAM_ATTRIBUTE, 0, ATTRIBUTE_ENUM, ATTRIBUTE_DISPLAYNAME, ATTRIBUTE_DESCRIPTION, false, PARAMETER_CATEGORY));
	return parlst;
}

uint tryGetSkeletonMeshIndex(MeshDocument const& m)
{
	for (uint i = 0; i < m.meshNumber(); i++) {
		auto mesh = m.getMesh(i);
		if (mesh->label().contains("skel", Qt::CaseSensitivity::CaseInsensitive))
			return i;
	}
	return 0;
}

uint tryGetTreeMeshIndex(MeshDocument const& m)
{
	for (uint i = 0; i < m.meshNumber(); i++) {
		auto mesh = m.getMesh(i);
		if (mesh->label().contains("tree", Qt::CaseSensitivity::CaseInsensitive))
			return i;
	}
	return 0;
}

uint tryGetOriginalMeshIndex(MeshDocument const& m)
{
	if (m.meshNumber() == 2)
	{
		auto skeleton = tryGetSkeletonMeshIndex(m);
		auto tree = tryGetTreeMeshIndex(m);
		if (skeleton != tree)
			return 3 - skeleton - tree;
		else
			return 0;
	}
	else
		return 0;
}

}
