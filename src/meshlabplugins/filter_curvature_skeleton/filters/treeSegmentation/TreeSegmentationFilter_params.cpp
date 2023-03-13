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

#include "TreeSegmentationFilter.h"

//defaults
#define DEFAULT_FACETAG "segmentation_tag"

// displayed strings
#define MESH_CATEGORY "(0) Meshes"
#define ORIGINAL_MESH_DISPLAYNAME "Original Mesh"
#define ORIGINAL_MESH_DESCRIPTION "The original mesh which has generated the <b>Skeleton</b> mesh"
#define SKELETON_MESH_DISPLAYNAME "Skeleton Mesh"
#define SKELETON_MESH_DESCRIPTION "The skeleton mesh."

#define PARAMETER_CATEGORY "(1) Hack Order Computing"
#define ROOT_SELECTED_DISPLAYNAME "Set root to selected vertex"
#define ROOT_SELECTED_DESCRIPTION "If selected, the root of the skeleton is set to the selected point instead of taking the lowest point on the Y axis."
#define SWAP_SELECTED_BRANCHES_DISPLAYNAME "Promote selected branch"
#define SWAP_SELECTED_BRANCHES_DESCRIPTION "Promote selected branch when computing the Hack numbers"

#define EXTRA_CATEGORY "(2) Extra"
#define MAP_TO_COLOR_DISPLAYNAME "Display Hack orders with colors."
#define MAP_TO_COLOR_DESCRIPTION "Display Hack orders with colors."
#define SAVE_GRAPH_DISPLAYNAME "Save generated graph as mesh."
#define SAVE_GRAPH_DESCRIPTION "Save generated graph as mesh."
#define FACE_TAG_ID_DISPLAYNAME "Face Tag additional attribute name."
#define FACE_TAG_ID_DESCRIPTION "Face Tag additional attribute name."

namespace curvatureSkeleton
{

static uint tryGetOriginalMeshIndex(MeshDocument const&);
static uint tryGetSkeletonMeshIndex(MeshDocument const&);

RichParameterList TreeSegmentationFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;

	auto* skeleton = m.getMesh(tryGetSkeletonMeshIndex(m));
	auto* original = m.getMesh(tryGetOriginalMeshIndex(m));

	parlst.addParam(RichMesh(PARAM_ORIGINAL_MESH, tryGetOriginalMeshIndex(m), &m, ORIGINAL_MESH_DISPLAYNAME, ORIGINAL_MESH_DESCRIPTION, false, MESH_CATEGORY));
	parlst.addParam(RichMesh(PARAM_SKELETON_MESH, tryGetSkeletonMeshIndex(m), &m, SKELETON_MESH_DISPLAYNAME, SKELETON_MESH_DESCRIPTION, false, MESH_CATEGORY));

	parlst.addParam(RichBool(PARAM_SELECTED_ROOT, skeleton->cm.svn == 1, ROOT_SELECTED_DISPLAYNAME, ROOT_SELECTED_DESCRIPTION, false, PARAMETER_CATEGORY));
	parlst.addParam(RichBool(PARAM_SELECTED_BRANCHES, original->cm.sfn > 1, SWAP_SELECTED_BRANCHES_DISPLAYNAME, SWAP_SELECTED_BRANCHES_DESCRIPTION, false, PARAMETER_CATEGORY));

	parlst.addParam(RichBool(PARAM_MAP_TO_COLOR, true, MAP_TO_COLOR_DISPLAYNAME, MAP_TO_COLOR_DESCRIPTION, false, EXTRA_CATEGORY));
	parlst.addParam(RichBool(PARAM_SAVE_GRAPH, false, SAVE_GRAPH_DISPLAYNAME, SAVE_GRAPH_DESCRIPTION, true, EXTRA_CATEGORY));
	parlst.addParam(RichString(PARAM_FACE_TAG_ID, DEFAULT_FACETAG, FACE_TAG_ID_DISPLAYNAME, FACE_TAG_ID_DESCRIPTION, true, EXTRA_CATEGORY));

	return parlst;
}

uint tryGetSkeletonMeshIndex(MeshDocument const& m)
{
	for (uint i = 0; i < m.meshNumber(); i++)
	{
		auto mesh = m.getMesh(i);
		if (mesh->label().contains("skel", Qt::CaseSensitivity::CaseInsensitive))
			return i;
	}
	return 0;
}

uint tryGetOriginalMeshIndex(MeshDocument const& m)
{
	if (m.meshNumber() == 2)
		return 1 - tryGetSkeletonMeshIndex(m);
	else
		return 0;
}

}
