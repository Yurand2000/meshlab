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

#include "BranchesCalculatorFilter.h"

// displayed strings
#define PARAMETER_CATEGORY "Parameters"
#define ROOT_INDEX_DISPLAYNAME "Root Vertex Index"
#define ROOT_INDEX_DESCRIPTION "The index of the root vertex of the skeleton tree, which defaults to the lowest point of the skeleton "\
							   "on the Y axis. If your model is aligned with the Y axis as the up direction, you probably don't need to change this value, "\
							   "else you might need to find the vertex index.\nTUTORIAL: You can get this index by using the \'Get Info\' tool, "\
							   "switching to \'Select Vertices\' mode, selecting the root vertex and logging its information to the console. "\
							   "Then copy the index number here."

namespace curvatureSkeleton
{

static uint findLowestVertexIndexOnY(CMeshO const& mesh);

RichParameterList BranchesCalculatorFilter::initParameterList(FilterPlugin const& p, MeshModel const& model)
{
	RichParameterList parlst;

	parlst.addParam(RichInt(PARAM_ROOT_INDEX, findLowestVertexIndexOnY(model.cm), ROOT_INDEX_DISPLAYNAME, ROOT_INDEX_DESCRIPTION, false, PARAMETER_CATEGORY));
	return parlst;
}

uint findLowestVertexIndexOnY(CMeshO const& mesh)
{
	uint    min_index = 0;
	Scalarm min_y     = std::numeric_limits<Scalarm>::max();
	for (uint i = 0; i < mesh.vert.size(); i++)
	{
		auto vert_y = mesh.vert[i].P().Y();
		if ( vert_y < min_y )
		{
			min_y = vert_y;
			min_index = i;
		}
	}

	return min_index;
}

}
