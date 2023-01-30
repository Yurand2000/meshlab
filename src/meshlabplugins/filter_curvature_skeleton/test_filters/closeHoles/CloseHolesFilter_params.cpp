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

#include "CloseHolesFilter.h"

#include <string>

namespace curvatureSkeleton
{

RichParameterList CloseHolesTestFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;

	parlst.addParam(RichInt(PARAM_MAX_HOLE_SIZE, (int)30, "Max size to be closed ", ""));
	parlst.addParam(RichBool(PARAM_SELECTED_FACES, m.mm()->cm.sfn > 0, "Close holes with selected faces", ""));
	parlst.addParam(RichBool(PARAM_SELECT_NEW_FACES, true, "Select the newly created faces", ""));
	parlst.addParam(RichBool(PARAM_PREVENT_SELF_INTERSECTION, true, "Prevent creation of selfIntersecting faces", ""));
	parlst.addParam(RichFloat(PARAM_DIEDRAL_WEIGHT, 0.1, "DIEDRAL WEIGHT", "", false));

	return parlst;
}

}
