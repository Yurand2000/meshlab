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

#include "ShowMinRadiusFilter.h"

#include <string>

namespace curvatureSkeleton
{

RichParameterList ShowMinRadiusTestFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;
	auto display_types = QStringList{
		"minimum radius",
		"average radius delta"
	};

	parlst.addParam(RichMesh(PARAM_ORIGINAL_MESH, 0, &m, "ORIGINAL MESH", "", false));
	parlst.addParam(RichMesh(PARAM_SKELETON_MESH, 0, &m, "SKELETON MESH", "", false));
	parlst.addParam(RichEnum(PARAM_DISPLAY_TYPE, 0, display_types, "DATA TO DISPLAY", "", false));

	return parlst;
}

}
