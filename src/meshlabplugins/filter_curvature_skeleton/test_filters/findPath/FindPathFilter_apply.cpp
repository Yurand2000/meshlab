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

#include "FindPathFilter.h"

#include "common/SkeletonMesh.h"
#include "common/EdgeMeshUtils.h"

namespace curvatureSkeleton
{

std::map<std::string, QVariant> FindPathTestFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& rich_params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos*		 cb)
{
	auto& mesh = document.mm()->cm;

	//clone to SkeletonMesh
	SkeletonMesh skeleton_mesh;
	vcg::tri::Append<SkeletonMesh, CMeshO>::MeshCopyConst(skeleton_mesh, mesh);
	vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(skeleton_mesh);
	vcg::tri::InitVertexIMark(skeleton_mesh);

	//get start and end positions
	int start_index = -1, end_index = -1;
	for (auto& vertex : mesh.vert)
	{
		if (vertex.IsS()) {
			if (start_index == -1)
				start_index = vertex.Index();
			else if (end_index == -1)
				end_index = vertex.Index();
			else
				throw MLException("More than two vertices selected!");
		}
	}

	if (start_index == -1 || end_index == -1) {
		throw MLException("Less than two vertices selected!");
	}

	auto* start_vertex = &skeleton_mesh.vert[start_index];
	auto* end_vertex = &skeleton_mesh.vert[end_index];

	auto path = EdgeMeshUtils<SkeletonMesh>::getVerticesPath(skeleton_mesh, start_vertex, end_vertex);

	for (auto* vertex : path) {
		mesh.vert[vertex->Index()].SetS();
	}

	return {};
}

}
