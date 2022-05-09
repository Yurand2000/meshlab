/****************************************************************************
 * MeshLab                                                           o o     *
 * An extendible mesh processor                                    o     o   *
 *                                                                _   O  _   *
 * Copyright(C) 2005 - 2020                                          \/)\/   *
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

#ifndef SKELETON_MESH
#define SKELETON_MESH

#include <vector>

#include <common/ml_document/base_types.h>
#include <common/ml_document/cmesh.h>

namespace curvatureSkeleton
{
	class SkeletonVertex;
	class SkeletonEdge;

	class SkeletonUsedTypes : public vcg::UsedTypes<
			vcg::Use<SkeletonVertex>::AsVertexType,
			vcg::Use<SkeletonEdge>::AsEdgeType
		>
	{ };

	class SkeletonVertex : public vcg::Vertex<SkeletonUsedTypes,
			vcg::vertex::InfoOcf,
			vcg::vertex::Coord3m,
			vcg::vertex::Normal3m,
			vcg::vertex::Qualitym,
			vcg::vertex::Color4b,
			vcg::vertex::VEAdj,
			vcg::vertex::BitFlags
		>
	{ };

	class SkeletonEdge : public vcg::Edge<SkeletonUsedTypes,
			vcg::edge::VEAdj,
			vcg::edge::EVAdj,
			vcg::edge::EEAdj,
			vcg::edge::BitFlags
		>
	{ };

	typedef vcg::tri::TriMesh<
		vcg::vertex::vector_ocf<SkeletonVertex>,
		std::vector<SkeletonEdge>
	> SkeletonMesh;
}

#endif //SKELETON_MESH
