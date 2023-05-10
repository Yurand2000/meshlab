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
#include "common/Utils.h"

//defaults
#define DEFAULT_FACETAG "segmentation_tag"

// displayed strings
#define INPUT_CATEGORY "(0) Input"
#define ORIGINAL_MESH_DISPLAYNAME "Mesh"
#define ORIGINAL_MESH_DESCRIPTION ""
#define SKELETON_MESH_DISPLAYNAME "Skeleton"
#define SKELETON_MESH_DESCRIPTION ""
#define ROOT_SELECTED_DISPLAYNAME "Set selected vertex as root"
#define ROOT_SELECTED_DESCRIPTION "Set the root of the skeleton as the selected point instead of the point with lowest Y coordinate."

#define PARAMETER_CATEGORY "(1) Modify Hack ordering"
#define SWAP_SELECTED_BRANCHES_DISPLAYNAME "Prioritize selected branches"
#define SWAP_SELECTED_BRANCHES_DESCRIPTION "It lowers the Hack order of the selected branch(es), i.e. it promotes the branches to be the continuation of their direct fathers."

#define SAVE_CATEGORY "(2) Visualize and save"
#define MAP_TO_COLOR_DISPLAYNAME "Display Hack ordering with colors"
#define MAP_TO_COLOR_DESCRIPTION ""
#define SAVE_GRAPH_DISPLAYNAME "Save Hack graph"
#define SAVE_GRAPH_DESCRIPTION "Save as mesh."
#define FACE_TAG_ID_DISPLAYNAME "Save face labels as an additional attribute"
#define FACE_TAG_ID_DESCRIPTION "Additional attribute name"

namespace curvatureSkeleton
{

RichParameterList TreeSegmentationFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;

	auto skeleton_index = 0, mesh_index = 0;
	auto selected_root = false, selected_branches = false;
	if (MeshDocumentUtils::tryGetSkeletonMeshIndex(m, skeleton_index) && MeshDocumentUtils::tryGetOriginalMeshIndex(m, mesh_index))
	{
		selected_root = m.getMesh(skeleton_index)->cm.svn == 1;
		selected_branches = m.getMesh(mesh_index)->cm.sfn > 1;
	}

	parlst.addParam(RichMesh(PARAM_ORIGINAL_MESH, mesh_index, &m, ORIGINAL_MESH_DISPLAYNAME, ORIGINAL_MESH_DESCRIPTION, false, INPUT_CATEGORY));
	parlst.addParam(RichMesh(PARAM_SKELETON_MESH, skeleton_index, &m, SKELETON_MESH_DISPLAYNAME, SKELETON_MESH_DESCRIPTION, false, INPUT_CATEGORY));
	parlst.addParam(RichBool(PARAM_SELECTED_ROOT, selected_root, ROOT_SELECTED_DISPLAYNAME, ROOT_SELECTED_DESCRIPTION, false, INPUT_CATEGORY));

	parlst.addParam(RichBool(PARAM_SELECTED_BRANCHES, selected_branches, SWAP_SELECTED_BRANCHES_DISPLAYNAME, SWAP_SELECTED_BRANCHES_DESCRIPTION, false, PARAMETER_CATEGORY));

	parlst.addParam(RichBool(PARAM_MAP_TO_COLOR, true, MAP_TO_COLOR_DISPLAYNAME, MAP_TO_COLOR_DESCRIPTION, false, SAVE_CATEGORY));
	parlst.addParam(RichBool(PARAM_SAVE_GRAPH, false, SAVE_GRAPH_DISPLAYNAME, SAVE_GRAPH_DESCRIPTION, true, SAVE_CATEGORY));
	parlst.addParam(RichString(PARAM_FACE_TAG_ID, DEFAULT_FACETAG, FACE_TAG_ID_DISPLAYNAME, FACE_TAG_ID_DESCRIPTION, true, SAVE_CATEGORY));

	return parlst;
}

}
