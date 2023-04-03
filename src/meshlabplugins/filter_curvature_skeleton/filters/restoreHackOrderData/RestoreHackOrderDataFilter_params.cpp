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

#include "RestoreHackOrderDataFilter.h"

 //defaults
#define DEFAULT_FACETAG "segmentation_tag"
#define DEFAULT_CLOSEHOLE_ADJ_FACETAG "hole_adj_segmentation_tag"

// displayed strings
#define MESH_CATEGORY "(0) Meshes"
#define ORIGINAL_MESH_DISPLAYNAME "Original Mesh"
#define ORIGINAL_MESH_DESCRIPTION "The original mesh which has generated the <b>Skeleton</b> mesh"

#define EXTRA_CATEGORY "(1) Extra"
#define FACE_TAG_ID_DISPLAYNAME "Face Tag attribute name"
#define FACE_TAG_ID_DESCRIPTION "Face Tag attribute name"
#define CLOSE_HOLES_ADJACENCY_TAG_ID_DISPLAYNAME "Close Hole Face Tag attribute name"
#define CLOSE_HOLES_ADJACENCY_TAG_ID_DESCRIPTION "Close Hole Face Tag attribute name"

namespace curvatureSkeleton
{

RichParameterList RestoreHackOrderDataFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;

	parlst.addParam(RichMesh(PARAM_ORIGINAL_MESH, m.mm()->id(), &m, ORIGINAL_MESH_DISPLAYNAME, ORIGINAL_MESH_DESCRIPTION, false, MESH_CATEGORY));
	parlst.addParam(RichString(PARAM_HOLE_ADJ_TAG_ID, DEFAULT_CLOSEHOLE_ADJ_FACETAG, CLOSE_HOLES_ADJACENCY_TAG_ID_DISPLAYNAME, CLOSE_HOLES_ADJACENCY_TAG_ID_DESCRIPTION, true, EXTRA_CATEGORY));
	parlst.addParam(RichString(PARAM_FACE_TAG_ID, DEFAULT_FACETAG, FACE_TAG_ID_DISPLAYNAME, FACE_TAG_ID_DESCRIPTION, true, EXTRA_CATEGORY));

	return parlst;
}

}
