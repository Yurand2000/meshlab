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

#include "RefinePolylineFilter.h"

namespace curvatureSkeleton
{

RichParameterList RefinePolylineTestFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;

	parlst.addParam(RichMesh(PARAM_ORIGINAL_MESH, 0, &m, "ORIGINAL MESH", ""));
	parlst.addParam(RichMesh(PARAM_POLYLINE_MESH, 0, &m, "POLYLINE MESH", ""));
	parlst.addParam(RichInt(PARAM_ITERATIONS, 1, "Iterations", ""));
	parlst.addParam(RichDynamicFloat(PARAM_SMOOTH_WEIGTH, 0.5f, 0.f, 1.f, "Smooth Weight", ""));
	parlst.addParam(RichDynamicFloat(PARAM_PROJECT_WEIGTH, 0.5f, 0.f, 1.f, "Project Weight", ""));

	return parlst;
}

}
