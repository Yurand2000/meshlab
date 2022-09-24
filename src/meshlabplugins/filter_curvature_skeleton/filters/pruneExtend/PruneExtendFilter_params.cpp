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

#include "PruneExtendFilter.h"

//default values
#define PRUNING_PERCENTILE_DEFAULT 10.f
#define DO_EXTEND_BRANCHES_DEFAULT true
#define CONE_ANGLE_DEFAULT Scalarm(10)

// displayed strings
#define MESH_CATEGORY "(0) Meshes"
#define ORIGINAL_MESH_DISPLAYNAME "Original Mesh"
#define ORIGINAL_MESH_DESCRIPTION "The original mesh which has generated the <b>Skeleton</b> mesh"
#define SKELETON_MESH_DISPLAYNAME "Skeleton Mesh"
#define SKELETON_MESH_DESCRIPTION "The skeleton mesh."

#define PARAMETER_CATEGORY "(1) Pruning"
#define ROOT_INDEX_DISPLAYNAME "Root Vertex Index"
#define ROOT_INDEX_DESCRIPTION "The index of the root vertex of the skeleton tree. Defaults to -1 meaning that the algorithm will use the lowest point "\
							   "on the Y axis of the skeleton. If your model is aligned with the Y axis as the up direction, you probably don't need to change this value, "\
							   "else you might need to find the vertex index.\nTUTORIAL: You can get this index by using the \'Get Info\' tool, "\
							   "switching to \'Select Vertices\' mode, selecting the root vertex and logging its information to the console. "\
							   "Then copy the index number here."
#define MIN_EDGE_SIZE_DISPLAYNAME "Minimum branch length"
#define MIN_EDGE_SIZE_DESCRIPTION "When generating the intermediate tree, branches or leafs shorter than the given length will be collapsed."
#define MIN_EDGE_PERCENTILE_DISPLAYNAME "Percentile of branches to crop"
#define MIN_EDGE_PERCENTILE_DESCRIPTION "When generating the intermediate tree, branches or leafs belonging to the given percentile on the lengths of the branches will be collapsed."

#define EXTEND_BRANCHES_CATEGORY "(2) Branch Extension"
#define EXTEND_BRANCHES_CONE_ANGLE_DISPLAYNAME "Cone Angle"
#define EXTEND_BRANCHES_CONE_ANGLE_DESCRIPTION "Extend the branches of the selected skeleton to the tip of the original mesh branch. "\
											   "The branch tip is found by taking the average of the points in a cone in the same direction as the branch normal vector."\
										       "This parameter selects the angle the cone spans from the branch normal, in degrees.\n"\
							                   "NOTE: the total angle of the cone will be double this angle!"

namespace curvatureSkeleton
{

static uint tryGetOriginalMeshIndex(MeshDocument const&);
static uint tryGetSkeletonMeshIndex(MeshDocument const&);

RichParameterList PruneExtendFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;

	auto skel_diagonal = m.getMesh(tryGetSkeletonMeshIndex(m))->cm.bbox.Diag();

	parlst.addParam(RichMesh(PARAM_ORIGINAL_MESH, tryGetOriginalMeshIndex(m), &m, ORIGINAL_MESH_DISPLAYNAME, ORIGINAL_MESH_DESCRIPTION, false, MESH_CATEGORY));
	parlst.addParam(RichMesh(PARAM_SKELETON_MESH, tryGetSkeletonMeshIndex(m), &m, SKELETON_MESH_DISPLAYNAME, SKELETON_MESH_DESCRIPTION, false, MESH_CATEGORY));
	parlst.addParam(RichInt(PARAM_ROOT_INDEX, -1, ROOT_INDEX_DISPLAYNAME, ROOT_INDEX_DESCRIPTION, false, PARAMETER_CATEGORY));

	parlst.addParam(RichPercentage(PARAM_MIN_EDGE_SIZE, skel_diagonal / 100.f, 0.f, skel_diagonal, MIN_EDGE_SIZE_DISPLAYNAME, MIN_EDGE_SIZE_DESCRIPTION, false, PARAMETER_CATEGORY));
	parlst.addParam(RichDynamicFloat(PARAM_MIN_EDGE_PERCENTILE, PRUNING_PERCENTILE_DEFAULT, 0.f, 100.f, MIN_EDGE_PERCENTILE_DISPLAYNAME, MIN_EDGE_PERCENTILE_DESCRIPTION, false, PARAMETER_CATEGORY));

	parlst.addParam(RichDynamicFloat(PARAM_EXTENSION_CONE_ANGLE, CONE_ANGLE_DEFAULT, 0, 90, EXTEND_BRANCHES_CONE_ANGLE_DISPLAYNAME, EXTEND_BRANCHES_CONE_ANGLE_DESCRIPTION, false, EXTEND_BRANCHES_CATEGORY));
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
