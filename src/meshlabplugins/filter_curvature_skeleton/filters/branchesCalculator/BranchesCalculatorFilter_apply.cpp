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

#include "simplifySkeleton/SimplifySkeleton.h"

namespace curvatureSkeleton
{

typedef vcg::tri::UpdateTopology<SkeletonMesh> SkeletonMeshTopology;
typedef vcg::tri::Allocator<CMeshO>            CMeshOAllocator;
typedef vcg::tri::Allocator<SkeletonMesh>      SkeletonAllocator;
typedef vcg::tri::Append<CMeshO, SkeletonMesh> SkeletonToCMeshOAppend;
typedef vcg::tri::Append<SkeletonMesh, CMeshO> CMeshOToSkeletonAppend;

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

	SkeletonMesh converted_skeleton;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_skeleton, skeleton);
	SkeletonMeshTopology::VertexEdge(converted_skeleton);

	if ( !isMeshConnected(converted_skeleton) )
		throw MLException("Given graph mesh must be connected.");
	collapseTwoConnectedVertices(converted_skeleton);
	collapseShortEdges(converted_skeleton, root_index, .1f);
	collapseTwoConnectedVertices(converted_skeleton);

	CMeshO new_skeleton;
	SkeletonToCMeshOAppend::MeshCopyConst(new_skeleton, converted_skeleton);
	auto mesh = document.addNewMesh(new_skeleton, "Tree-" + document.mm()->label(), false);
	mesh->clearDataMask(MeshModel::MM_VERTQUALITY);

	return {};
}

}
