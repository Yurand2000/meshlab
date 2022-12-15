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

#ifndef POLYLINE_MESH
#define POLYLINE_MESH

#include <vector>

#include <common/ml_document/base_types.h>
#include <common/ml_document/cmesh.h>

namespace curvatureSkeleton
{
	class PolylineVertex;
	class PolylineEdge;
	class PolylineFace;

	class PolylineUsedTypes : public vcg::UsedTypes<
		vcg::Use<PolylineVertex>::AsVertexType,
		vcg::Use<PolylineEdge>::AsEdgeType,
		vcg::Use<PolylineFace>::AsFaceType
	>
	{ };

	class PolylineVertex : public vcg::Vertex<PolylineUsedTypes,
		vcg::vertex::InfoOcf,
		vcg::vertex::Coord3m,
		vcg::vertex::Normal3m,
		vcg::vertex::Qualitym,
		vcg::vertex::VEAdj,
		vcg::vertex::Mark,
		vcg::vertex::BitFlags
	>
	{ };

	class PolylineEdge : public vcg::Edge<PolylineUsedTypes,
		vcg::edge::VEAdj,
		vcg::edge::EVAdj,
		vcg::edge::EEAdj,
		vcg::edge::Mark,
		vcg::edge::BitFlags
	>
	{ };

	class PolylineFace : public vcg::Face<PolylineUsedTypes,
		vcg::face::VertexRef,
		vcg::face::Normal3m,
		vcg::face::FFAdj,
		vcg::face::VFAdj,
		vcg::face::Mark,
		vcg::face::BitFlags
	>
	{ };

	typedef vcg::tri::TriMesh<
		std::vector<PolylineVertex>,
		std::vector<PolylineEdge>,
		std::vector<PolylineFace>
	> PolylineMesh;
}

#endif //POLYLINE_MESH
