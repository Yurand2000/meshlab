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

#include "SkeletonizeFilter.h"
#include "filter_curvature_skeleton.h"

#define F_FILTERID	  FilterCurvatureSkeleton::SKELETONIZE
#define F_DISPLAYNAME "Skeletonize"
#define F_DESCRIPTION "Generate a 1D skeletal representation of a given mesh, " \
"according to the algorithm described in the referenced paper and implemented in the CGAL library. " \
"You may want to apply isotropic remeshing before running the filter." \
"<br /><br /><b>REFERENCES:</b><br />Tagliasacchi A., Alhashim I., Olson M., Zhang H.: <b>Mean Curvature Skeletons.</b><br />" \
"<i>In Computer Graphics Forum (Proc. of the Symposium on Geometry Processing) 31, 5 (2012), 1735-1744.</i><br />"\
"<a href='https://doi.org/10.1111/j.1467-8659.2012.03178.x'>doi:10.1111/j.1467-8659.2012.03178.x</a>"
#define F_CATEGORY    FilterPlugin::Other
#define F_PYTHON_NAME "skeletonize_mesh"
#define F_ARITY       FilterPlugin::FilterArity::FIXED
#define F_PRECONDS    MeshModel::MM_NONE
#define F_POSTCONDS   MeshModel::MM_VERTQUALITY

namespace curvatureSkeleton
{

SkeletonizeFilter::SkeletonizeFilter() :
	TemplateFilter(
		F_FILTERID, F_DISPLAYNAME, F_DESCRIPTION, F_CATEGORY,
		F_PYTHON_NAME, F_ARITY, F_PRECONDS, F_POSTCONDS) { }

}
