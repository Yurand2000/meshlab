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
#define F_DESCRIPTION "Perform the skeletonization of the given mesh. Some of the parameters can be adjusted to get finer or faster results."\
" The most important parameters are edge collapsing length, which determines the overall resolution of the resulting skeleton and"\
" the quality speed and medially centering tradeoff values, which control the convergence speed at the cost of the quality of the resulting skeleton.<br /><br />"\
" The parameters referenced in the paper are <b>Omega_L, Omega_H, Omega_M, and Epsilon</b>. The <b>Omega_L</b> parameter is considered always as 1 and it is not needed"\
" because the three Omega values have a partition of unity property over multiplication. The other two Omega parameters are called respectively"\
" <b>Quality-Speed Tradeoff</b> and <b>Medially Centering Tradeoff</b>. The <b>Epsilon</b> parameter refers instead the <b>Min Edge Length</b> parameter." \
" The angle of the opposing vertex at which an edge is split is not meant as a parameter by the paper, here instead it can be changed."\
" It performs the algorithm described in the referenced paper and implemented into the CGAL library."\
"<br /><br /><b>REFERENCES:</b><br />Tagliasacchi A., Alhashim I., Olson M., Zhang H.: <b>Mean Curvature Skeletons.</b><br />"\
"<i>In Computer Graphics Forum (Proc. of the Symposium on Geometry Processing) 31, 5 (2012), 1735-1744.</i><br /><a href='https://doi.org/10.1111/j.1467-8659.2012.03178.x'>doi:10.1111/j.1467-8659.2012.03178.x</a>"
#define F_CATEGORY    FilterPlugin::Other
#define F_PYTHON_NAME "skeletonizer_mesh"
#define F_ARITY       FilterPlugin::FilterArity::SINGLE_MESH
#define F_PRECONDS    MeshModel::MM_NONE
#define F_POSTCONDS   MeshModel::MM_VERTQUALITY

namespace curvatureSkeleton
{

SkeletonizeFilter::SkeletonizeFilter() :
	TemplateFilter(
		F_FILTERID, F_DISPLAYNAME, F_DESCRIPTION, F_CATEGORY,
		F_PYTHON_NAME, F_ARITY, F_PRECONDS, F_POSTCONDS) { }

}
