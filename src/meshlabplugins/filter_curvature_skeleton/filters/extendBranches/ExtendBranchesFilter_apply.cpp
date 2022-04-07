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

#include "ExtendBranchesFilter.h"

#include "branchExtender/BranchExtender.h"

#define _USE_MATH_DEFINES
#include <cmath>

namespace curvatureSkeleton
{

std::map<std::string, QVariant> ExtendBranchesFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos* callback)
{
	auto& skeleton = document.getMesh( params.getMeshId(PARAM_SKELETON_MESH) )->cm;
	auto const& original = document.getMesh( params.getMeshId(PARAM_ORIGINAL_MESH) )->cm;
	auto cone_angle = (params.getDynamicFloat(PARAM_CONE_ANGLE) * Scalarm(M_PI)) / Scalarm(180.0);

	extendLeafBranches(original, skeleton, cone_angle);

	return {};
}

}
