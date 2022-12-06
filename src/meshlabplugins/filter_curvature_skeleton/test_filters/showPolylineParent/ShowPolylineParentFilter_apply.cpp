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

#include "ShowPolylineParentFilter.h"

#include <common/PolylineMesh.h>

#define ATTRIBUTE_BRANCH_NUMBER "branch_number"

namespace curvatureSkeleton
{
static CMeshO extractSkeletonBranch(CMeshO& skeleton, int branch_num);

std::map<std::string, QVariant> ShowPolylineParentTestFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& rich_params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos*		 cb)
{
	auto& polylines = document.getMesh(rich_params.getMeshId(PARAM_POLYLINE_MESH))->cm;
	auto& skeleton = document.getMesh(rich_params.getMeshId(PARAM_SKELETON_MESH))->cm;

	//convert meshes to PolylineMesh
	PolylineMesh converted_mesh, converted_polylines;
	vcg::tri::Allocator<PolylineMesh>::AddPerVertexAttribute<Scalarm>(converted_polylines, ATTRIBUTE_BRANCH_NUMBER);
	vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopy(converted_polylines, polylines);

	//split polyline in connected components
	std::vector<std::pair<int, PolylineMesh::EdgeType*>> polylines_data;
	vcg::tri::Clean<PolylineMesh>::edgeMeshConnectedComponents(converted_polylines, polylines_data);

	PolylineMesh polyline;
	auto parent_branch_num = vcg::tri::Allocator<PolylineMesh>::AddPerVertexAttribute<Scalarm>(polyline, ATTRIBUTE_BRANCH_NUMBER);
	int i = 0;
	for (auto& polyline_data : polylines_data)
	{
		//generated mesh
		auto* newmesh = document.addNewMesh("", QString::asprintf("Polyline And Parent %d", i), false);
		newmesh->setVisible(false);

		//extract polyline
		auto it = vcg::tri::EdgeConnectedComponentIterator<PolylineMesh>();
		vcg::tri::UpdateSelection<PolylineMesh>::Clear(converted_polylines);
		for (it.start(converted_polylines, polyline_data.second); !it.completed(); ++it) {
			(*it)->SetS();
		}
		vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshCopy(polyline, converted_polylines, true);
		vcg::tri::UpdateSelection<PolylineMesh>::Clear(polyline);

		//append polyline
		vcg::tri::Append<CMeshO, PolylineMesh>::MeshAppendConst(newmesh->cm, polyline);

		//extract parent branch
		auto parent = extractSkeletonBranch(skeleton, parent_branch_num[0]);

		//append parent
		vcg::tri::Append<CMeshO, CMeshO>::MeshAppendConst(newmesh->cm, parent);

		i++;
	}

	return {};
}

CMeshO extractSkeletonBranch(CMeshO& skeleton, int branch_num)
{
	CMeshO branch;

	auto poly_number = vcg::tri::Allocator<CMeshO>::FindPerVertexAttribute<Scalarm>(skeleton, ATTRIBUTE_BRANCH_NUMBER);
	if ( !vcg::tri::Allocator<CMeshO>::IsValidHandle(skeleton, poly_number) ) {
		throw new MLException("ATTRIBUTE MISSING ERROR!");
	}

	for (auto& vertex : skeleton.vert) {
		if (poly_number[vertex] == branch_num) {
			vertex.SetS();
		}
	}

	vcg::tri::Append<CMeshO, CMeshO>::MeshCopyConst(branch, skeleton, true);

	vcg::tri::UpdateSelection<CMeshO>::Clear(skeleton);
	vcg::tri::UpdateSelection<CMeshO>::Clear(branch);
	return branch;
}

}
