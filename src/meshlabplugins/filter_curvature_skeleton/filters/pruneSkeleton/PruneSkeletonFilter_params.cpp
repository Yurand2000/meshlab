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

#include "PruneSkeletonFilter.h"
#include "common/Utils.h"

// displayed strings
#define INPUT_CATEGORY "(0) Input"
#define ORIGINAL_MESH_DISPLAYNAME "Mesh"
#define ORIGINAL_MESH_DESCRIPTION ""
#define SKELETON_MESH_DISPLAYNAME "Skeleton"
#define SKELETON_MESH_DESCRIPTION ""

#define LENGTH_PRUNING_CATEGORY "(1) Pruning by length"
#define MIN_EDGE_LENGHT_DISPLAYNAME "Branch length"
#define MIN_EDGE_LENGHT_DESCRIPTION "Remove branches whose length is below threshold (absolute length and percentage of bbox diagonal)."

#define SELECTION_PRUNING_CATEGORY "(2) Pruning by selection"
#define REMOVE_SELECTED_LEAFS_DISPLAYNAME "Prune selected branches"
#define REMOVE_SELECTED_LEAFS_DESCRIPTION ""

namespace curvatureSkeleton
{

RichParameterList PruneSkeletonFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;

	auto skeleton_index = 0, mesh_index = 0;
	Scalarm average_edge_lenght = 0, skeleton_diagonal = -1.0;
	bool is_any_vertex_selected = false;
	if (MeshDocumentUtils::tryGetSkeletonMeshIndex(m, skeleton_index) && MeshDocumentUtils::tryGetOriginalMeshIndex(m, mesh_index))
	{
		auto* skeleton = m.getMesh(skeleton_index);
		skeleton_diagonal = skeleton->cm.bbox.Diag();

		is_any_vertex_selected = skeleton->cm.svn > 0;

		for (auto& edge : skeleton->cm.edge)
			average_edge_lenght += vcg::edge::Length(edge);
		average_edge_lenght /= skeleton->cm.EN();
	}

	parlst.addParam(RichMesh(PARAM_ORIGINAL_MESH, mesh_index, &m, ORIGINAL_MESH_DISPLAYNAME, ORIGINAL_MESH_DESCRIPTION, false, INPUT_CATEGORY));
	parlst.addParam(RichMesh(PARAM_SKELETON_MESH, skeleton_index, &m, SKELETON_MESH_DISPLAYNAME, SKELETON_MESH_DESCRIPTION, false, INPUT_CATEGORY));

	if (skeleton_diagonal >= 0)
		parlst.addParam(RichPercentage(PARAM_MIN_EDGE_LENGHT, 3.f * average_edge_lenght, 0.f, skeleton_diagonal, MIN_EDGE_LENGHT_DISPLAYNAME, MIN_EDGE_LENGHT_DESCRIPTION, false, LENGTH_PRUNING_CATEGORY));
	else
		parlst.addParam(RichFloat(PARAM_MIN_EDGE_LENGHT, 0.0, MIN_EDGE_LENGHT_DISPLAYNAME, MIN_EDGE_LENGHT_DESCRIPTION, false, LENGTH_PRUNING_CATEGORY));
	parlst.addParam(RichBool(PARAM_REMOVE_SELECTED_LEAFS, is_any_vertex_selected, REMOVE_SELECTED_LEAFS_DISPLAYNAME, REMOVE_SELECTED_LEAFS_DESCRIPTION, false, SELECTION_PRUNING_CATEGORY));

	return parlst;
}

}
