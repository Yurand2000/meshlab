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

#include"strahlerBranchTagging/SkeletonTreeBuilder.h"

namespace curvatureSkeleton
{

std::map<std::string, QVariant> BranchesCalculatorFilter::applyFilter(
	FilterPlugin const& plugin,
	RichParameterList const& params,
	MeshDocument& document,
	unsigned int&,
	vcg::CallBackPos* cb)
{
	auto& skeleton  = document.mm()->cm;
	auto root_index = params.getInt(PARAM_ROOT_INDEX);
	if (root_index < 0 || root_index >= skeleton.vert.size())
		throw MLException("Given index does not represent any valid vertex on the selected mesh.");

	SkeletonTreeBuilder::checkSkeletonTree(document.mm()->cm);

	auto mesh = document.addNewMesh(CMeshO(), "Tree-" + document.mm()->label(), false);
	SkeletonTreeBuilder::generateTree(mesh->cm, document.mm()->cm, static_cast<uint>(root_index));

	mesh->clearDataMask(MeshModel::MM_VERTQUALITY);

	return {};
}

}
