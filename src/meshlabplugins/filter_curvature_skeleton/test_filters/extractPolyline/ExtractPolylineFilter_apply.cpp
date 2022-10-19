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

#include "ExtractPolylineFilter.h"

#include "common/MeshBorderPolyline.h"

namespace curvatureSkeleton
{

std::map<std::string, QVariant> ExtractPolylineTestFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& rich_params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos*		 cb)
{
	auto& mesh = document.mm()->cm;

	mesh.face.EnableFFAdjacency();
	auto polyline = MeshBorderPolyline<CMeshO>::getLongestPolyline(mesh);
	mesh.face.DisableFFAdjacency();

	auto* newmesh = document.addNewMesh("", QString::asprintf("Polyline"), false);
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopyConst(newmesh->cm, polyline);
	newmesh->setVisible(false);

	return {};
}

}
