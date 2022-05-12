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
#define MESH_CATEGORY "Meshes"
#define ORIGINAL_MESH_DISPLAYNAME "Original Mesh"
#define ORIGINAL_MESH_DESCRIPTION "The original mesh which has generated the <b>Skeleton</b> mesh"
#define SKELETON_MESH_DISPLAYNAME "Skeleton Mesh"
#define SKELETON_MESH_DESCRIPTION "The skeleton mesh."
#define PARAMETER_CATEGORY "Parameters"
#define ROOT_INDEX_DISPLAYNAME "Root Vertex Index"
#define ROOT_INDEX_DESCRIPTION "The index of the root vertex of the skeleton tree. Defaults to -1 meaning that the algorithm will use the lowest point "\
							   "on the Y axis of the skeleton. If your model is aligned with the Y axis as the up direction, you probably don't need to change this value, "\
							   "else you might need to find the vertex index.\nTUTORIAL: You can get this index by using the \'Get Info\' tool, "\
							   "switching to \'Select Vertices\' mode, selecting the root vertex and logging its information to the console. "\
							   "Then copy the index number here."
#define MIN_EDGE_SIZE_DISPLAYNAME "Percentile of branches to crop."
#define MIN_EDGE_SIZE_DESCRIPTION "When generating the intermediate tree, branches or leafs belonging to the given percentile on the lengths of the branches will be collapsed."
#define STRAHLER_TO_COLOR_DISPLAYNAME "Colorize by the Strahler number"
#define STRAHLER_TO_COLOR_DESCRIPTION "The meshes' vertices will be colorized by their Strahler number. "\
									  "You may need to toggle on and off the visualization to update the displayed colors."
#define MISCELLANEOUS_CATEGORY "Miscellaneous"
#define SAVE_GENERATED_TREE_DISPLAYNAME "Save generated tree"
#define SAVE_GENERATED_TREE_DESCRIPTION "Save the intermediate tree mesh used to calculate the Strahler Numbers."

namespace curvatureSkeleton
{

static uint getSelectedMeshIndex(MeshDocument const&);

RichParameterList StrahlerTaggingFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;

	parlst.addParam(RichMesh(PARAM_ORIGINAL_MESH, 0, &m, ORIGINAL_MESH_DISPLAYNAME, ORIGINAL_MESH_DESCRIPTION, false, MESH_CATEGORY));
	parlst.addParam(RichMesh(PARAM_SKELETON_MESH, getSelectedMeshIndex(m), &m, SKELETON_MESH_DISPLAYNAME, SKELETON_MESH_DESCRIPTION, false, MESH_CATEGORY));
	parlst.addParam(RichInt(PARAM_ROOT_INDEX, -1, ROOT_INDEX_DISPLAYNAME, ROOT_INDEX_DESCRIPTION, false, PARAMETER_CATEGORY));
	parlst.addParam(RichFloat(PARAM_MIN_EDGE_SIZE, 10.f, MIN_EDGE_SIZE_DISPLAYNAME, MIN_EDGE_SIZE_DESCRIPTION, false, PARAMETER_CATEGORY));
	parlst.addParam(RichBool(PARAM_STRAHLER_NUMBERS_TO_COLOR, true, STRAHLER_TO_COLOR_DISPLAYNAME, STRAHLER_TO_COLOR_DESCRIPTION, false, PARAMETER_CATEGORY));
	parlst.addParam(RichBool(PARAM_SAVE_GENERATED_TREE, true, SAVE_GENERATED_TREE_DISPLAYNAME, SAVE_GENERATED_TREE_DESCRIPTION, false, MISCELLANEOUS_CATEGORY));
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
