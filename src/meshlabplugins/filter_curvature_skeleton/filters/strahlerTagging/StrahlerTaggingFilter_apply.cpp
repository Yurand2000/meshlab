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

#include"strahlerBranchTagging/StrahlerBranchTagger.h"

namespace curvatureSkeleton
{

std::map<std::string, QVariant> StrahlerTaggingFilter::applyFilter(
	FilterPlugin const& plugin,
	RichParameterList const& params,
	MeshDocument& document,
	unsigned int&,
	vcg::CallBackPos* cb)
{
	auto original = document.getMesh(params.getMeshId(PARAM_ORIGINAL_MESH));
	auto skeleton = document.getMesh(params.getMeshId(PARAM_SKELETON_MESH));
	auto tree = document.getMesh( params.getMeshId(PARAM_TREE_MESH) );
	auto save_to_quality = params.getBool(PARAM_STRAHLER_NUMBERS_TO_QUALITY);

	StrahlerBranchTagger::calculateStrahlerNumbers(original->cm, skeleton->cm, tree->cm);
	original->setMeshModified(true);
	skeleton->setMeshModified(true);

	if (save_to_quality)
	{
		original->updateDataMask(MeshModel::MeshElement::MM_VERTQUALITY);
		skeleton->updateDataMask(MeshModel::MeshElement::MM_VERTQUALITY);
		StrahlerBranchTagger::strahlerNumberToQuality(skeleton->cm);
		StrahlerBranchTagger::strahlerNumberToQuality(original->cm);
	}

	return {};
}

}
