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

// displayed strings
#define MESH_CATEGORY "(0) Meshes"
#define ORIGINAL_MESH_DISPLAYNAME "Original Mesh"
#define ORIGINAL_MESH_DESCRIPTION "The original mesh which has generated the <b>Skeleton</b> mesh"
#define SKELETON_MESH_DISPLAYNAME "Skeleton Mesh"
#define SKELETON_MESH_DESCRIPTION "The skeleton mesh."

#define PARAMETER_CATEGORY "(1) Pruning"
#define MIN_EDGE_LENGHT_DISPLAYNAME "Minimum edge lenght"
#define MIN_EDGE_LENGHT_DESCRIPTION "Minimum edge lenght"
#define MIN_EDGE_PERCENTILE_DISPLAYNAME "Percentile of branches to crop"
#define MIN_EDGE_PERCENTILE_DESCRIPTION "When generating the intermediate tree, branches or leafs belonging to the given percentile on the lengths of the branches will be collapsed."
#define REMOVE_SELECTED_LEAFS_DISPLAYNAME "Remove selected branches"
#define REMOVE_SELECTED_LEAFS_DESCRIPTION "Remove selected branches"

namespace curvatureSkeleton
{

static uint tryGetOriginalMeshIndex(MeshDocument const&);
static uint tryGetSkeletonMeshIndex(MeshDocument const&);

RichParameterList PruneSkeletonFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;

	auto* skeleton = m.getMesh(tryGetSkeletonMeshIndex(m));
	auto skel_diagonal = skeleton->cm.bbox.Diag();

	auto is_any_vertex_selected = skeleton->cm.svn > 0;

	Scalarm average_edge_lenght = 0;
	for (auto& edge : skeleton->cm.edge)
		average_edge_lenght += vcg::edge::Length(edge);
	average_edge_lenght /= skeleton->cm.EN();

	parlst.addParam(RichMesh(PARAM_ORIGINAL_MESH, tryGetOriginalMeshIndex(m), &m, ORIGINAL_MESH_DISPLAYNAME, ORIGINAL_MESH_DESCRIPTION, false, MESH_CATEGORY));
	parlst.addParam(RichMesh(PARAM_SKELETON_MESH, tryGetSkeletonMeshIndex(m), &m, SKELETON_MESH_DISPLAYNAME, SKELETON_MESH_DESCRIPTION, false, MESH_CATEGORY));

	parlst.addParam(RichPercentage(PARAM_MIN_EDGE_LENGHT, 3.f * average_edge_lenght, 0.f, skel_diagonal, MIN_EDGE_LENGHT_DISPLAYNAME, MIN_EDGE_LENGHT_DESCRIPTION, false, PARAMETER_CATEGORY));
	parlst.addParam(RichDynamicFloat(PARAM_MIN_EDGE_PERCENTILE, 0.f, 0.f, skel_diagonal, MIN_EDGE_PERCENTILE_DISPLAYNAME, MIN_EDGE_PERCENTILE_DESCRIPTION, false, PARAMETER_CATEGORY));
	parlst.addParam(RichBool(PARAM_REMOVE_SELECTED_LEAFS, is_any_vertex_selected, REMOVE_SELECTED_LEAFS_DISPLAYNAME, REMOVE_SELECTED_LEAFS_DESCRIPTION, false, PARAMETER_CATEGORY));

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
