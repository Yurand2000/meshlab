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
		    vcg::vertex::Mark,
			vcg::vertex::BitFlags
		>
	{ };

	class SkeletonEdge : public vcg::Edge<SkeletonUsedTypes,
			vcg::edge::VEAdj,
			vcg::edge::EVAdj,
			vcg::edge::EEAdj,
			vcg::edge::Mark,
			vcg::edge::BitFlags
		>
	{ };

	using SkeletonMeshBase = vcg::tri::TriMesh<
		vcg::vertex::vector_ocf<SkeletonVertex>,
		std::vector<SkeletonEdge>
	>;

	class SkeletonMesh : public SkeletonMeshBase {
	public:
		SkeletonMesh() : SkeletonMeshBase() {}

		//SkeletonMesh(const SkeletonMesh& oth);
		SkeletonMesh(SkeletonMesh&& other) : SkeletonMeshBase() {
			swap(*this, other);
		}

		SkeletonMesh& operator=(SkeletonMesh&& other) {
			if (&other != this)
				swap(*this, other);
			return *this;
		}

	private:
		inline void swap(SkeletonMesh& m1, SkeletonMesh& m2)
		{
			std::swap(m1.vn, m2.vn);
			std::swap(m1.vert, m2.vert);
			std::swap(m1.en, m2.en);
			std::swap(m1.edge, m2.edge);
			std::swap(m1.fn, m2.fn);
			std::swap(m1.face, m2.face);
			std::swap(m1.hn, m2.hn);
			std::swap(m1.hedge, m2.hedge);
			std::swap(m1.tn, m2.tn);
			std::swap(m1.tetra, m2.tetra);
			std::swap(m1.bbox, m2.bbox);
			std::swap(m1.textures, m2.textures);
			std::swap(m1.normalmaps, m2.normalmaps);
			std::swap(m1.attrn, m2.attrn);
			std::swap(m1.vert_attr, m2.vert_attr);
			std::swap(m1.edge_attr, m2.edge_attr);
			std::swap(m1.face_attr, m2.face_attr);
			std::swap(m1.mesh_attr, m2.mesh_attr);
			std::swap(m1.tetra_attr, m2.tetra_attr);
			std::swap(m1.shot, m2.shot);
			std::swap(m1.imark, m2.imark);
		}
	};
}

#endif //SKELETON_MESH
