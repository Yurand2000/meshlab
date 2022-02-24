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

#include "skeletonize_default.h"
#include "algorithm_skeletonize.h"
#include "../filter_curvature_skeleton.h"

#define F_FILTERID	  FilterCurvatureSkeleton::SKELETONIZE_DEFAULT
#define F_DISPLAYNAME "Skeletonize Mesh (Default settings)"
#define F_DESCRIPTION "Generate the skeleton of the selected mesh, using default settings. The given mesh must be watertight for it to be applied. For more settings and info use the filter <b>Skeletonize Manual</b>."
#define F_CATEGORY    FilterPlugin::Other
#define F_PYTHON_NAME "skeletonizer_mesh_default"
#define F_ARITY       FilterPlugin::FilterArity::SINGLE_MESH
#define F_PRECONDS    MeshModel::MM_NONE
#define F_POSTCONDS   MeshModel::MM_NONE

filterSkeletonizeDefault::filterSkeletonizeDefault() :
	templateFilter(
		F_FILTERID, F_DISPLAYNAME, F_DESCRIPTION, F_CATEGORY,
		F_PYTHON_NAME, F_ARITY, F_PRECONDS, F_POSTCONDS) { }

filterSkeletonizeDefault::~filterSkeletonizeDefault() { }

RichParameterList filterSkeletonizeDefault::initParameterList(FilterPlugin const&, MeshModel const& m)
{
	return RichParameterList();
}

#define F_DEFAULT_ITERATIONS 500
#define F_DEFAULT_GENERATE_MESO_SKELETONS false
#define F_DEFAULT_SAVE_DISTANCE_FROM_SKELETON false

std::map<std::string, QVariant> filterSkeletonizeDefault::applyFilter(
	FilterPlugin const& plugin,
	RichParameterList const& params,
	MeshDocument& document,
	unsigned int&,
	vcg::CallBackPos* callback)
{
	auto skel_params = algorithmSkeletonize::Parameters(document.mm()->cm);
	return algorithmSkeletonize(document, skel_params, *callback, plugin).apply(
			F_DEFAULT_ITERATIONS,
			F_DEFAULT_GENERATE_MESO_SKELETONS,
			F_DEFAULT_SAVE_DISTANCE_FROM_SKELETON
		);
}
