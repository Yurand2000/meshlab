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
		vcg::vertex::Color4b,
		vcg::vertex::VEAdj,
		vcg::vertex::VFAdj,
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

	class PolylineMesh : public vcg::tri::TriMesh<
		std::vector<PolylineVertex>,
		std::vector<PolylineEdge>,
		std::vector<PolylineFace>
	>
	{

	public:
		PolylineMesh() = default;
		PolylineMesh(PolylineMesh&& other) {
			swap(*this, other);
		}
		PolylineMesh& operator=(PolylineMesh&& other) {
			if (&other != this)
				swap(*this, other);
			return *this;
		}

	private:
		inline void swap(PolylineMesh& m1, PolylineMesh& m2)
		{
			using std::swap;
			swap(m1.vn, m2.vn);
			swap(m1.vert, m2.vert);
			swap(m1.en, m2.en);
			swap(m1.edge, m2.edge);
			swap(m1.fn, m2.fn);
			swap(m1.face, m2.face);
			swap(m1.hn, m2.hn);
			swap(m1.hedge, m2.hedge);
			swap(m1.tn, m2.tn);
			swap(m1.tetra, m2.tetra);
			swap(m1.bbox, m2.bbox);
			swap(m1.textures, m2.textures);
			swap(m1.normalmaps, m2.normalmaps);
			swap(m1.attrn, m2.attrn);
			swap(m1.vert_attr, m2.vert_attr);
			swap(m1.edge_attr, m2.edge_attr);
			swap(m1.face_attr, m2.face_attr);
			swap(m1.mesh_attr, m2.mesh_attr);
			swap(m1.tetra_attr, m2.tetra_attr);
			swap(m1.shot, m2.shot);
			swap(m1.imark, m2.imark);
		}
	};
}

#endif //POLYLINE_MESH
