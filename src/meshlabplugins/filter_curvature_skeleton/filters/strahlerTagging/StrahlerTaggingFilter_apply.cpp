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

#include "strahlerBranchTagging/StrahlerBranchTagger.h"
#include <vcg/complex/algorithms/update/color.h>

typedef vcg::tri::UpdateColor<CMeshO> UpdateColor;
typedef vcg::Color4b                  Color;

namespace curvatureSkeleton {

static void strahlerNumberToVertexColor(CMeshO& mesh);
static void strahlerNumberToVertexColor(
	StrahlerBranchTagger::StrahlerVertexNumbers const& numbers,
	CMeshO&                                            mesh);

std::map<std::string, QVariant> StrahlerTaggingFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos* cb)
{
	auto original        = document.getMesh(params.getMeshId(PARAM_ORIGINAL_MESH));
	auto skeleton        = document.getMesh(params.getMeshId(PARAM_SKELETON_MESH));
	auto tree            = document.getMesh(params.getMeshId(PARAM_TREE_MESH));
	auto save_to_quality = params.getBool(PARAM_STRAHLER_NUMBERS_TO_QUALITY);
	auto save_to_color   = params.getBool(PARAM_STRAHLER_NUMBERS_TO_COLOR);

	StrahlerBranchTagger::calculateStrahlerNumbers(original->cm, skeleton->cm, tree->cm);
	original->setMeshModified(true);
	skeleton->setMeshModified(true);
	tree->setMeshModified(true);

	if (save_to_quality)
	{
		original->updateDataMask(MeshModel::MeshElement::MM_VERTQUALITY);
		skeleton->updateDataMask(MeshModel::MeshElement::MM_VERTQUALITY);
		tree->updateDataMask(MeshModel::MeshElement::MM_VERTQUALITY);

		StrahlerBranchTagger::strahlerNumberToQuality(skeleton->cm);
		StrahlerBranchTagger::strahlerNumberToQuality(original->cm);
	}

	if (save_to_color)
	{
		original->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
		strahlerNumberToVertexColor(original->cm);

		skeleton->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
		strahlerNumberToVertexColor(skeleton->cm);

		tree->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
		auto tree_numbers = StrahlerBranchTagger::getNodeNumbers(tree->cm);
		strahlerNumberToVertexColor(tree_numbers, tree->cm);
	}

	return {};
}

void strahlerNumberToVertexColor(CMeshO& mesh)
{
	auto numbers = StrahlerBranchTagger::getStrahlerNumbers(mesh);
	strahlerNumberToVertexColor(numbers, mesh);
}

void strahlerNumberToVertexColor(
	StrahlerBranchTagger::StrahlerVertexNumbers const& numbers,
	CMeshO&                                            mesh)
{
	uint min = 1, max = 1;
	for (int i = 0; i < mesh.VN(); i++) {
		if (numbers[i] > max)
			max = numbers[i];
	}

	uint range  = max;
	auto colors = std::vector<Color>(range);
	for (uint i = 0; i < range; i++) {
		colors[i] = Color::Scatter(range, i, 0.6f, 1.f);
	}

	for (int i = 0; i < mesh.VN(); i++) {
		auto& vertex   = mesh.vert[i];
		auto  strahler = numbers[i] - min;
		vertex.C()     = colors[strahler];
	}
}

}
