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

#include "PruneExtendFilter.h"

#include "common/SkeletonMesh.h"
#include "common/BranchTagger.h"
#include "common/BranchExtender.h"
#include "common/Utils.h"
#include <vcg/complex/algorithms/update/color.h>
#include <vcg/complex/algorithms/stat.h>

namespace curvatureSkeleton
{

typedef vcg::tri::Allocator<CMeshO>            CMeshOAllocator;
typedef vcg::tri::Allocator<SkeletonMesh>      SkeletonAllocator;
typedef vcg::tri::Append<CMeshO, CMeshO> CMeshOToCMeshOAppend;
typedef vcg::tri::Append<SkeletonMesh, CMeshO> CMeshOToSkeletonAppend;
typedef vcg::tri::Append<CMeshO, SkeletonMesh> SkeletonToCMeshOAppend;
typedef vcg::tri::UpdateColor<CMeshO>          UpdateColor;
typedef vcg::Color4b                           Color;

#define ATTRIBUTE_ROOT_INDEX "root_index"

static bool isVertexValid(int index, CMeshO const& skeleton);
static int getLowestPointOnYAxis(CMeshO const& mesh);
static int findVertexIndex(SkeletonMesh const& tree, vcg::Point3<Scalarm> root_pos);

std::map<std::string, QVariant> PruneExtendFilter::applyFilter(
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
	auto  root_index    = params.getInt(PARAM_ROOT_INDEX);
	auto  min_edge_size = params.getAbsPerc(PARAM_MIN_EDGE_SIZE);
	auto  min_edge_perc = params.getDynamicFloat(PARAM_MIN_EDGE_PERCENTILE);
	auto  extend_angle  = params.getDynamicFloat(PARAM_EXTENSION_CONE_ANGLE);
	
	if ( !isVertexValid(root_index, skeleton) )
		root_index = getLowestPointOnYAxis(skeleton);

	//generate the pruned tree
	SkeletonMesh converted_tree, converted_skeleton;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_skeleton, skeleton);
	BranchTagger<SkeletonMesh>::generateTreeMesh(converted_tree, converted_skeleton, root_index, min_edge_size, min_edge_perc);

	//create a copy of the skeleton mesh
	auto skeleton_copy_mm = document.addNewMesh(CMeshO(), "Pruned-" + skeleton_mm->label(), false);
	skeleton_copy_mm->clearDataMask(MeshModel::MeshElement::MM_VERTQUALITY);
	auto& skeleton_copy = skeleton_copy_mm->cm;
	CMeshOToCMeshOAppend::MeshCopyConst(skeleton_copy, skeleton);
	CMeshOAllocator::GetPerMeshAttribute<Scalarm>(skeleton_copy, ATTRIBUTE_ROOT_INDEX)() = root_index;

	//extend only the relevant branches
	for (auto const& vertex : converted_tree.vert)
	{
		if (vcg::edge::VEDegree<SkeletonEdge>(&vertex) == 1)
		{
			auto skeleton_index = Utils<CMeshO>::getVertexIndexInMesh(vertex.cP(), skeleton_copy);
			if(skeleton_index != root_index)
				BranchExtender::extendBranch(original, skeleton_copy, skeleton_index, extend_angle);
		}
	}

	//re-generate the pruned tree
	CMeshOToSkeletonAppend::MeshCopyConst(converted_skeleton, skeleton_copy);
	BranchTagger<SkeletonMesh>::generateTreeMesh(converted_tree, converted_skeleton, root_index, min_edge_size, min_edge_perc);

	int  tree_root_index = findVertexIndex(
		converted_tree,
		skeleton.vert[root_index].cP()
	);

	//save the generated tree as a mesh
	auto tree_mm = document.addNewMesh(CMeshO(), "Tree-" + original_mm->label(), false);
	tree_mm->clearDataMask(MeshModel::MeshElement::MM_VERTQUALITY);
	auto& tree = tree_mm->cm;
	SkeletonToCMeshOAppend::MeshCopyConst(tree, converted_tree);
	CMeshOAllocator::GetPerMeshAttribute<Scalarm>(tree, ATTRIBUTE_ROOT_INDEX)() = tree_root_index;

	return {};
}

bool isVertexValid(int index, CMeshO const& skeleton)
{
	return index >= 0 &&
		   index < skeleton.vert.size() &&
		   !skeleton.vert[index].IsD();
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

int findVertexIndex(SkeletonMesh const& tree, vcg::Point3<Scalarm> vertex_pos)
{
	for (int i = 0; i < tree.vert.size(); i++)
	{
		auto& vertex = tree.vert[i];
		if (!vertex.IsD() && (vcg::SquaredDistance(vertex.cP(), vertex_pos) < 0.0001f))
		{
			return i;
		}
	}
	return 0;
}

}
