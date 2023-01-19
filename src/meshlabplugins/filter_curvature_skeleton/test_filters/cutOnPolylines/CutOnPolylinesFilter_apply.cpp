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

#include "CutOnPolylinesFilter.h"

#include "common/PolylineMesh.h"
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/curve_on_manifold.h>
#include <vcg/complex/algorithms/crease_cut.h>

#define ATTRIBUTE_BRANCH_NUMBER "branch_number"

namespace curvatureSkeleton
{

std::map<std::string, QVariant> CutOnPolylinesTestFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& rich_params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos*		 cb)
{
	auto& original = document.getMesh( rich_params.getMeshId(PARAM_ORIGINAL_MESH) )->cm;
	auto& polylines_mesh = document.getMesh( rich_params.getMeshId(PARAM_POLYLINE_MESH) )->cm;

	//convert meshes to PolylineMesh
	PolylineMesh converted_mesh, converted_polylines;
	vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopy(converted_mesh, original); //COPY ATTRIBUTES?
	vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopy(converted_polylines, polylines_mesh);
	vcg::tri::UpdateTopology<PolylineMesh>::FaceFace(converted_mesh);
	vcg::tri::UpdateTopology<PolylineMesh>::VertexFace(converted_mesh);
	vcg::tri::UpdateTopology<PolylineMesh>::VertexEdge(converted_mesh);

	//prepare curve on manifold class
	auto com = vcg::tri::CoM<PolylineMesh>(converted_mesh);
	com.Init();

	//split polyline in connected components
	std::vector<std::pair<int, PolylineMesh::EdgeType*>> polylines_data;
	vcg::tri::Clean<PolylineMesh>::edgeMeshConnectedComponents(converted_polylines, polylines_data);

	PolylineMesh polyline;
	vcg::tri::Allocator<PolylineMesh>::AddPerVertexAttribute<Scalarm>(polyline, ATTRIBUTE_BRANCH_NUMBER);
	auto parent_branch_numbers = vcg::tri::Allocator<PolylineMesh>::GetPerVertexAttribute<Scalarm>(converted_polylines, ATTRIBUTE_BRANCH_NUMBER);
	for (auto& polyline_data : polylines_data)
	{
		//extract polyline
		auto it = vcg::tri::EdgeConnectedComponentIterator<PolylineMesh>();
		vcg::tri::UpdateSelection<PolylineMesh>::EdgeClear(converted_polylines);
		for (it.start(converted_polylines, polyline_data.second); !it.completed(); ++it) {
			(*it)->SetS();
		}
		vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshCopy(polyline, converted_polylines, true);
		vcg::tri::UpdateTopology<PolylineMesh>::VertexEdge(polyline);
		vcg::tri::UpdateSelection<PolylineMesh>::Clear(polyline);

		//cut base mesh on polyline
		vcg::tri::UpdateSelection<PolylineMesh>::Clear(converted_mesh);
		com.RefineCurveByBaseMesh(polyline);
		com.SplitMeshWithPolyline(polyline);
		com.TagFaceEdgeSelWithPolyLine(polyline);
		CutMeshAlongSelectedFaceEdges(converted_mesh);
	}

	//reconvert base_mesh to CMeshO
	CMeshO cut_original;
	vcg::tri::Append<CMeshO, PolylineMesh>::MeshCopy(cut_original, converted_mesh);

	//split in connected components
	std::vector<std::pair<int, CFaceO*>> conn_comps;

	cut_original.face.EnableFFAdjacency();
	vcg::tri::UpdateTopology<CMeshO>::FaceFace(cut_original);
	vcg::tri::Clean<CMeshO>::ConnectedComponents(cut_original, conn_comps);

	//save connected components
	for (int i = 0; i < conn_comps.size(); i++)
	{
		auto& pair = conn_comps[i];
		auto new_mesh = document.addNewMesh("", QString::asprintf("Branch %d", i), false); //make numbering consistent
		new_mesh->setVisible(false);
		pair.second->SetS();
		vcg::tri::UpdateSelection<CMeshO>::FaceConnectedFF(cut_original);
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(new_mesh->cm, cut_original, true);
		vcg::tri::UpdateSelection<CMeshO>::Clear(new_mesh->cm);
		vcg::tri::UpdateSelection<CMeshO>::Clear(cut_original);
	}

	return {};
}

}
