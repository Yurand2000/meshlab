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

#include "SkeletonTreeBuilder.h"

#include "simplifySkeleton/SimplifySkeleton.h"
#include <common/mlexception.h>
#include <vector>

namespace curvatureSkeleton
{

typedef vcg::tri::UpdateTopology<SkeletonMesh> SkeletonMeshTopology;
typedef vcg::tri::Allocator<CMeshO>            CMeshOAllocator;
typedef vcg::tri::Allocator<SkeletonMesh>      SkeletonAllocator;
typedef vcg::tri::Append<SkeletonMesh, SkeletonMesh> SkeletonToSkeletonAppend;
typedef vcg::tri::Append<SkeletonMesh, CMeshO> CMeshOToSkeletonAppend;

void generateTreeMesh(SkeletonMesh& tree, CMeshO const& skeleton, int root_index, Scalarm percentile)
{
	if (root_index < 0 || root_index >= skeleton.vert.size())
		throw MLException("Given index does not represent any valid vertex on the selected mesh.");

	SkeletonMesh converted_skeleton;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_skeleton, skeleton);
	SkeletonMeshTopology::VertexEdge(converted_skeleton);

	if (!isMeshConnected(converted_skeleton))
		throw MLException("Given graph mesh must be connected.");

	collapseTwoConnectedVertices(converted_skeleton);
	auto percentile_length = calculateEdgeLengthPercentile(converted_skeleton, percentile);
	collapseShortEdges(converted_skeleton, root_index, percentile_length);
	collapseTwoConnectedVertices(converted_skeleton);

	SkeletonToSkeletonAppend::MeshCopyConst(tree, converted_skeleton);
	SkeletonMeshTopology::VertexEdge(tree);
}

Scalarm calculateEdgeLengthPercentile(SkeletonMesh const& mesh, Scalarm percentile)
{
	std::vector<Scalarm> lengths;
	lengths.reserve( mesh.EN() );

	for (auto& edge : mesh.edge)
	{
		if ( !edge.IsD() )
			lengths.push_back( (edge.V(0)->cP() - edge.V(1)->cP()).SquaredNorm() );
	}

	std::sort(lengths.begin(), lengths.end());
	uint percentile_pos = static_cast<uint>(  std::ceil( (lengths.size() - 1) * percentile / 100.f )  );
	return std::sqrt( lengths[percentile_pos] );
}

}
