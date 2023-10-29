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

#include "PruneSkeletonFilter.h"

#include "common/SkeletonMesh.h"
#include "common/PruneSkeleton.h"

namespace curvatureSkeleton
{

std::map<std::string, QVariant> PruneSkeletonFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos* cb)
{
	auto  original_mm = document.getMesh(params.getMeshId(PARAM_ORIGINAL_MESH));
	auto& original = original_mm->cm;
	auto  skeleton_mm = document.getMesh(params.getMeshId(PARAM_SKELETON_MESH));
	auto& skeleton = skeleton_mm->cm;

	auto  min_branch_lenght = params.getDynamicFloat(PARAM_MIN_EDGE_LENGHT);
	auto  remove_selected = params.getBool(PARAM_REMOVE_SELECTED_LEAFS);

	//convert to skeleton mesh
	SkeletonMesh c_skeleton;
	vcg::tri::Append<SkeletonMesh, CMeshO>::MeshCopyConst(c_skeleton, skeleton);

	PruneSkeleton::pruneSkeleton(original, c_skeleton, min_branch_lenght, remove_selected);

	//save updated skeleton
	vcg::tri::Append<CMeshO, SkeletonMesh>::MeshCopy(skeleton, c_skeleton);

	return {};
}

}
