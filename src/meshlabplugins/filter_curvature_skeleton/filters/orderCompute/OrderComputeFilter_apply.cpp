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

#include "OrderComputeFilter.h"

#include "common/SkeletonMesh.h"
#include "common/BranchTagger.h"
#include <vcg/complex/algorithms/update/color.h>
#include <vcg/complex/algorithms/stat.h>

#define ATTRIBUTE_STRAHLER_NUMBER "strahler_number"

namespace curvatureSkeleton
{

typedef vcg::tri::Allocator<CMeshO>            CMeshOAllocator;
typedef vcg::tri::Allocator<SkeletonMesh>      SkeletonAllocator;
typedef vcg::tri::Append<SkeletonMesh, CMeshO> CMeshOToSkeletonAppend;
typedef vcg::tri::Append<CMeshO, SkeletonMesh> SkeletonToCMeshOAppend;
typedef vcg::tri::UpdateColor<CMeshO>          UpdateColor;
typedef vcg::Color4b                           Color;

static int getLowestPointOnYAxis(CMeshO const& mesh);
static int findRootIndex(SkeletonMesh const& tree, vcg::Point3<Scalarm> root_pos);

std::map<std::string, QVariant> OrderComputeFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos* cb)
{
	auto  original_mm   = document.getMesh(params.getMeshId(PARAM_ORIGINAL_MESH));
	auto& original      = original_mm->cm;
	auto  skeleton_mm   = document.getMesh(params.getMeshId(PARAM_SKELETON_MESH));
	auto& skeleton      = skeleton_mm->cm;
	auto  save_to_color = params.getBool(PARAM_STRAHLER_NUMBERS_TO_COLOR);
	auto  save_gen_tree = params.getBool(PARAM_SAVE_GENERATED_TREE);
	auto  root_index    = params.getInt(PARAM_ROOT_INDEX);
	auto  min_edge_size = params.getFloat(PARAM_MIN_EDGE_SIZE);
	
    //if root_index is not defined, get the lowest point on the Y axis
	if (root_index < 0)
		root_index = getLowestPointOnYAxis(skeleton);

	//generate the logical tree
	SkeletonMesh converted_tree, converted_skeleton;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_skeleton, skeleton);
	BranchTagger<SkeletonMesh>::generateTreeMesh(converted_tree, converted_skeleton, root_index, min_edge_size);
	vcg::tri::InitVertexIMark(converted_tree);
	vcg::tri::InitEdgeIMark(converted_tree);

	//compute numbers
	auto root_pos        = skeleton.vert[root_index].cP();
	int  tree_root_index = findRootIndex(converted_tree, root_pos);

	vcg::tri::Stat<SkeletonMesh>::CalculateStrahlerNumbers(converted_tree, tree_root_index);

	//generate tree mesh
	CMeshO tree;
	{
		auto numbers = CMeshOAllocator::AddPerVertexAttribute<Scalarm>(tree, ATTRIBUTE_STRAHLER_NUMBER);
		SkeletonToCMeshOAppend::MeshCopyConst(tree, converted_tree);
	}

	//save attributes back to the original meshes
	BranchTagger<SkeletonMesh>::treeScalarAttributeToSkeleton(converted_skeleton, converted_tree, ATTRIBUTE_STRAHLER_NUMBER);

	{
		auto numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(skeleton, ATTRIBUTE_STRAHLER_NUMBER);
		auto converted_numbers = SkeletonAllocator::GetPerVertexAttribute<Scalarm>(converted_skeleton, ATTRIBUTE_STRAHLER_NUMBER);
		for (int i = 0; i < skeleton.VN(); i++)
			numbers[i] = converted_numbers[i];
	}

	BranchTagger<CMeshO>::skeletonScalarAttributeToOriginalMesh(original, skeleton, ATTRIBUTE_STRAHLER_NUMBER);

	if (save_to_color)
	{
		auto colors = BranchTagger<CMeshO>::generateColorsFromAttribute(tree, ATTRIBUTE_STRAHLER_NUMBER);

		skeleton_mm->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
		BranchTagger<CMeshO>::colorizeByAttribute(skeleton, colors, ATTRIBUTE_STRAHLER_NUMBER);

		original_mm->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
		BranchTagger<CMeshO>::colorizeByAttribute(original, colors, ATTRIBUTE_STRAHLER_NUMBER);
	}

	if (save_gen_tree)
	{
		auto mesh = document.addNewMesh(tree, "Tree-" + original_mm->label(), false);
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

int getLowestPointOnYAxis(CMeshO const& mesh)
{
	int index = 0;

	Scalarm min_y = std::numeric_limits<Scalarm>::max();
	for (auto& vert : mesh.vert)
	{
		if (vert.P().Y() < min_y)
		{
			min_y = vert.P().Y();
			index = vert.Index();
		}
	}

	return index;
}

}
