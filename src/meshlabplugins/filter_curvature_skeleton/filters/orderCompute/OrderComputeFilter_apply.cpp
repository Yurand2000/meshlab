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
#define ATTRIBUTE_HACK_ORDER_NUMBER "hack_order_number"
#define ATTRIBUTE_ROOT_INDEX "root_index"
#define ATTRIBUTE_SKELETON_TO_TREE "tree_index"
#define ATTRIBUTE_MESH_TO_SKELETON "skeleton_index"

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
	auto  save_to_color = params.getEnum(PARAM_ATTRIBUTE_TO_COLOR);
	auto  root_index    = params.getInt(PARAM_ROOT_INDEX);
	auto  min_edge_size = params.getAbsPerc(PARAM_MIN_EDGE_SIZE);
	auto  min_edge_perc = params.getDynamicFloat(PARAM_MIN_EDGE_PERCENTILE);
	
    //if root_index is not defined, get the lowest point on the Y axis
	if ( root_index < 0 || root_index > skeleton.vert.size() || skeleton.vert[root_index].IsD() )
		root_index = getLowestPointOnYAxis(skeleton);

	//generate the logical tree
	SkeletonMesh converted_tree, converted_skeleton;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_skeleton, skeleton);
	BranchTagger<SkeletonMesh>::generateTreeMesh(converted_tree, converted_skeleton, root_index, min_edge_size, min_edge_perc);
	vcg::tri::InitVertexIMark(converted_tree);
	vcg::tri::InitEdgeIMark(converted_tree);

	auto root_pos        = skeleton.vert[root_index].cP();
	int  tree_root_index = findRootIndex(converted_tree, root_pos);

	//compute numbers
	vcg::tri::Stat<SkeletonMesh>::ComputeStrahlerNumbers(converted_tree, tree_root_index, ATTRIBUTE_STRAHLER_NUMBER);
	vcg::tri::Stat<SkeletonMesh>::ComputeHackOrderNumbers(converted_tree, tree_root_index, ATTRIBUTE_HACK_ORDER_NUMBER);

	//generate tree mesh
	auto  temp = CMeshO();
	auto  tree_mm = document.addNewMesh(temp, original_mm->label() + "-tree", false);
	tree_mm->clearDataMask(MeshModel::MeshElement::MM_VERTQUALITY);
	auto& tree = tree_mm->cm;
	{
		CMeshOAllocator::AddPerVertexAttribute<Scalarm>(tree, ATTRIBUTE_STRAHLER_NUMBER);
		CMeshOAllocator::AddPerVertexAttribute<Scalarm>(tree, ATTRIBUTE_HACK_ORDER_NUMBER);
		SkeletonToCMeshOAppend::MeshCopyConst(tree, converted_tree);
	}

	//save root index attribute
	CMeshOAllocator::GetPerMeshAttribute<Scalarm>(tree, ATTRIBUTE_ROOT_INDEX)() = tree_root_index;
	CMeshOAllocator::GetPerMeshAttribute<Scalarm>(skeleton, ATTRIBUTE_ROOT_INDEX)() = root_index;

	//save attributes back to the original meshes
	BranchTagger<CMeshO>::copyAttributeTreeToSkeleton(skeleton, tree, tree_root_index, ATTRIBUTE_STRAHLER_NUMBER, false);
	BranchTagger<CMeshO>::copyAttributeTreeToSkeleton(skeleton, tree, tree_root_index, ATTRIBUTE_HACK_ORDER_NUMBER, true);

	BranchTagger<CMeshO>::copyAttributeUsingAdjacency(skeleton, original, ATTRIBUTE_STRAHLER_NUMBER, ATTRIBUTE_MESH_TO_SKELETON);
	BranchTagger<CMeshO>::copyAttributeUsingAdjacency(skeleton, original, ATTRIBUTE_HACK_ORDER_NUMBER, ATTRIBUTE_MESH_TO_SKELETON);

	//colorize
	auto color_attrib = (save_to_color == 1) ? ATTRIBUTE_HACK_ORDER_NUMBER : ATTRIBUTE_STRAHLER_NUMBER;
	if (save_to_color > 0)
	{
		auto colors = BranchTagger<CMeshO>::generateColorsFromAttribute(tree, color_attrib);

		tree_mm->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
		BranchTagger<CMeshO>::colorizeByAttribute(tree, colors, color_attrib);

		skeleton_mm->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
		BranchTagger<CMeshO>::colorizeByAttribute(skeleton, colors, color_attrib);

		original_mm->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
		BranchTagger<CMeshO>::colorizeByAttribute(original, colors, color_attrib);
	}

	return {};
}

int findRootIndex(SkeletonMesh const& tree, vcg::Point3<Scalarm> root_pos)
{
	for (int i = 0; i < tree.vert.size(); i++)
	{
		auto& vertex = tree.vert[i];
		if ( !vertex.IsD() && ( vcg::SquaredDistance(vertex.cP(), root_pos) < 0.0001f ) )
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
		if ( !vert.IsD() && vert.P().Y() < min_y )
		{
			min_y = vert.P().Y();
			index = vert.Index();
		}
	}

	return index;
}

}
