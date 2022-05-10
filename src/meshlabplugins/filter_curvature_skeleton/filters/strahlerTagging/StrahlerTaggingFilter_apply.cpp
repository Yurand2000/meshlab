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

#include "common/StrahlerBranchTagger.h"
#include <vcg/complex/algorithms/update/color.h>

namespace curvatureSkeleton
{

typedef vcg::tri::Allocator<CMeshO>            CMeshOAllocator;
typedef vcg::tri::Allocator<SkeletonMesh>      SkeletonAllocator;
typedef vcg::tri::Append<CMeshO, SkeletonMesh> SkeletonToCMeshOAppend;
typedef vcg::tri::UpdateColor<CMeshO>          UpdateColor;
typedef vcg::Color4b                           Color;

static int findRootIndex(SkeletonMesh const& tree, vcg::Point3<Scalarm> root_pos);
static std::vector<Color> makeStrahlerColors(SkeletonMesh const& tree_mesh);
static void strahlerNumberToVertexColor(CMeshO& mesh, std::vector<Color> const& colors);

std::map<std::string, QVariant> StrahlerTaggingFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos* cb)
{
	auto original      = document.getMesh(params.getMeshId(PARAM_ORIGINAL_MESH));
	auto skeleton      = document.getMesh(params.getMeshId(PARAM_SKELETON_MESH));
	auto save_to_color = params.getBool(PARAM_STRAHLER_NUMBERS_TO_COLOR);
	auto save_gen_tree = params.getBool(PARAM_SAVE_GENERATED_TREE);
	auto root_index    = params.getInt(PARAM_ROOT_INDEX);
	auto min_edge_size = params.getFloat(PARAM_MIN_EDGE_SIZE);

	SkeletonMesh tree;
	StrahlerBranchTagger::generateTreeMesh(tree, skeleton->cm, root_index, min_edge_size);
	vcg::tri::InitVertexIMark(tree);
	vcg::tri::InitEdgeIMark(tree);

	auto root_pos = skeleton->cm.vert[root_index].cP();
	int  tree_root_index = findRootIndex(tree, root_pos);

	StrahlerBranchTagger::calculateStrahlerNumbers(tree, tree_root_index);
	StrahlerBranchTagger::strahlerNumbersToSkeleton(skeleton->cm, tree, root_index);
	StrahlerBranchTagger::strahlerNumbersToOriginalMesh(original->cm, skeleton->cm);

	if (save_to_color)
	{
		auto colors = makeStrahlerColors(tree);

		original->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
		strahlerNumberToVertexColor(original->cm, colors);

		skeleton->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
		strahlerNumberToVertexColor(skeleton->cm, colors);
	}

	if (save_gen_tree)
	{
		CMeshO tree_mesh;
		auto numbers = CMeshOAllocator::AddPerVertexAttribute<uint>(tree_mesh, ATTRIBUTE_STRAHLER_NUMBER);
		SkeletonToCMeshOAppend::MeshCopyConst(tree_mesh, tree);
		auto mesh = document.addNewMesh(tree_mesh, "Tree-" + original->label(), false);
		mesh->clearDataMask(MeshModel::MeshElement::MM_VERTQUALITY);
	}

	return {};
}

int findRootIndex(SkeletonMesh const& tree, vcg::Point3<Scalarm> root_pos)
{
	for (int i = 0; i < tree.vert.size(); i++)
	{
		if ( (tree.vert[i].cP() - root_pos).SquaredNorm() < 0.0001f )
		{
			return i;
		}
	}
	return 0;
}

std::vector<Color> makeStrahlerColors(SkeletonMesh const& tree_mesh)
{
	auto tree_numbers = SkeletonAllocator::GetPerVertexAttribute<uint>(tree_mesh, ATTRIBUTE_STRAHLER_NUMBER);

	uint min = 1, max = 1;
	for (int i = 0; i < tree_mesh.VN(); i++)
	{
		if (tree_numbers[i] > max)
			max = tree_numbers[i];
	}

	uint range  = max - min + 1;
	auto colors = std::vector<Color>(range + min);
	for (uint i = 0; i < range; i++)
	{
		colors[i + min] = Color::Scatter(range, i, 0.6f, 1.f);
	}

	return colors;
}

void strahlerNumberToVertexColor(CMeshO& mesh, std::vector<Color> const& colors)
{
	auto numbers = CMeshOAllocator::GetPerVertexAttribute<uint>(mesh, ATTRIBUTE_STRAHLER_NUMBER);
	for (int i = 0; i < mesh.VN(); i++)
	{
		auto& vertex = mesh.vert[i];
		vertex.C()   = colors[numbers[i]];
	}
}

}
