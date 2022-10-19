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

#include "RefinePolylineFilter.h"

#include "common/PolylineMesh.h"
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/curve_on_manifold.h>

namespace curvatureSkeleton
{

std::map<std::string, QVariant> RefinePolylineTestFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& rich_params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos*		 cb)
{
	auto& mesh = document.getMesh( rich_params.getMeshId(PARAM_ORIGINAL_MESH) )->cm;
	auto& polylines_mesh = document.getMesh( rich_params.getMeshId(PARAM_POLYLINE_MESH) )->cm;
	auto iter = rich_params.getInt(PARAM_ITERATIONS); if (iter < 1) { iter = 1; }
	auto smooth_w = rich_params.getDynamicFloat(PARAM_SMOOTH_WEIGTH);
	auto proj_w = rich_params.getDynamicFloat(PARAM_PROJECT_WEIGTH);

	//convert meshes to SkeletonMesh
	PolylineMesh converted_mesh, converted_polylines;
	vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopy(converted_mesh, mesh);
	vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopy(converted_polylines, polylines_mesh);

	//prepare curve on manifold class
	auto com = vcg::tri::CoM<PolylineMesh>(converted_mesh);
	com.Init();

	//split polyline in connected components
	std::vector<std::pair<int, PolylineMesh::EdgeType*>> polylines_data;
	vcg::tri::Clean<PolylineMesh>::edgeMeshConnectedComponents(converted_polylines, polylines_data);

	PolylineMesh smoothed_polylines, polyline;
	for (auto& polyline_data : polylines_data)
	{
		//extract polyline
		auto it = vcg::tri::EdgeConnectedComponentIterator<PolylineMesh>();
		vcg::tri::UpdateSelection<PolylineMesh>::EdgeClear(converted_polylines);
		for (it.start(converted_polylines, polyline_data.second); !it.completed(); ++it) {
			(*it)->SetS();
		}
		vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshCopy(polyline, converted_polylines, true);
		vcg::tri::UpdateSelection<PolylineMesh>::VertexClear(polyline);
		vcg::tri::UpdateSelection<PolylineMesh>::EdgeClear(polyline);

		//smooth project polyline
		com.SmoothProject(polyline, iter, smooth_w, proj_w);

		//append polyline
		vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshAppendConst(smoothed_polylines, polyline);
	}

	vcg::tri::Append<CMeshO, PolylineMesh>::MeshCopy(polylines_mesh, smoothed_polylines);
	vcg::tri::UpdateSelection<CMeshO>::Clear(polylines_mesh);

	return {};
}

}
