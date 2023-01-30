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

#include "CloseHolesFilter.h"

#include <vcg/complex/algorithms/hole.h>

namespace curvatureSkeleton
{

std::map<std::string, QVariant> CloseHolesTestFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& rich_params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos*		 cb)
{
	auto& mesh_model = *document.mm();
	auto& mesh = mesh_model.cm;
	auto max_hole_size = rich_params.getInt(PARAM_MAX_HOLE_SIZE);
	auto diedral_weight = rich_params.getFloat(PARAM_DIEDRAL_WEIGHT);
	auto selected_faces = rich_params.getBool(PARAM_SELECTED_FACES);
	auto select_new_faces = rich_params.getBool(PARAM_SELECT_NEW_FACES);
	auto prevent_self_intersection = rich_params.getBool(PARAM_PREVENT_SELF_INTERSECTION);

	mesh_model.updateDataMask(MeshModel::MM_FACEFACETOPO);
	if (vcg::tri::Clean<CMeshO>::CountNonManifoldEdgeFF(mesh) > 0) {
		throw MLException("Mesh has some not 2-manifold edges, filter requires edge manifoldness");
	}

	vcg::tri::MinimumWeightEar<CMeshO>::DiedralWeight() = diedral_weight;

	size_t OriginalSize = mesh.face.size();
	int holeCnt;
	if (prevent_self_intersection) {
		holeCnt = vcg::tri::Hole<CMeshO>::EarCuttingIntersectionFill<vcg::tri::SelfIntersectionEar< CMeshO> >(mesh, max_hole_size, selected_faces, cb);
	}
	else {
		holeCnt = vcg::tri::Hole<CMeshO>::EarCuttingFill<vcg::tri::MinimumWeightEar< CMeshO> >(mesh, max_hole_size, selected_faces, cb);
	}
	plugin.log("Closed %i holes and added %i new faces", holeCnt, mesh.fn - OriginalSize);
	assert(vcg::tri::Clean<CMeshO>::IsFFAdjacencyConsistent(mesh));
	mesh_model.updateBoxAndNormals();

	if (select_new_faces)
	{
		vcg::tri::UpdateSelection<CMeshO>::FaceClear(mesh);
		for (size_t i = OriginalSize; i < mesh.face.size(); ++i)
			if (!mesh.face[i].IsD()) mesh.face[i].SetS();
	}

	return {};
}

}
