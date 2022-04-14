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

#include "StrahlerTaggingFilter.h"
#include "filter_curvature_skeleton.h"

#define F_FILTERID    FilterCurvatureSkeleton::CALCULATE_STRAHLER_NUMBERS
#define F_DISPLAYNAME "Calculate Strahler Numbers"
#define F_DESCRIPTION ""
#define F_CATEGORY    FilterPlugin::Other
#define F_PYTHON_NAME "calculate_strahler_numbers"
#define F_ARITY       FilterPlugin::FilterArity::FIXED
#define F_PRECONDS    MeshModel::MM_NONE
#define F_POSTCONDS   MeshModel::MM_NONE

namespace curvatureSkeleton
{

StrahlerTaggingFilter::StrahlerTaggingFilter()
	: TemplateFilter(
		F_FILTERID, F_DISPLAYNAME, F_DESCRIPTION, F_CATEGORY,
		F_PYTHON_NAME, F_ARITY, F_PRECONDS, F_POSTCONDS) { }

}
