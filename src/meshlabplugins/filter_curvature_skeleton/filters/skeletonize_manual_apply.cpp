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

#include "skeletonize_manual.h"
#include "cgal_adapter/mesh_skeletonizer.h"
#include "algo_skeletonize/skeletonize.h"

#include <string>

std::map<std::string, QVariant> filterSkeletonizeManual::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos* callback)
{
	try {
		auto& selected_mesh = document.mm()->cm;

		callback(100, "Setup - Starting...");
		checkParameters(params, *callback);
		updateBorderFlags(selected_mesh, *callback);
		checkSelectedMesh(selected_mesh, *callback);
		callback(100, "Setup - Done.");

		return algorithmSkeletonize(document, params, *callback, plugin).apply();
	}
	catch (MLException e) {
		throw e;
	}
	catch (std::exception e) {
		throw MLException(QString("Unhandled exception: ") + QString(e.what()));
	}
}


void filterSkeletonizeManual::checkParameters(RichParameterList const& params, vcg::CallBackPos& callback)
{
	callback(0, "Setup - Checking Parameters...");
	if (params.getInt(PARAM_MAX_ITERATIONS) < 1)
	{
		throw MLException("Number of iterations cannot be negative.");
	}

	if (params.getBool(PARAM_USE_DELTA_AREA_TERMINATION) &&
		params.getFloat(PARAM_DELTA_AREA_TERMINATION) <= 0) {
		throw MLException("Delta area convergence cannot be zero or negative.");
	}

	if (params.getBool(PARAM_USE_MAX_ANGLE) &&
		params.getFloat(PARAM_MAX_ANGLE) <= 0)
	{
		throw MLException("Max triangle angle cannot be zero or negative.");
	}

	if (params.getBool(PARAM_USE_MIN_EDGE_LENGTH) &&
		params.getFloat(PARAM_MIN_EDGE_LENGTH) <= 0)
	{
		throw MLException("Min edge length cannot be zero or negative.");
	}

	if (params.getBool(PARAM_USE_QUALITY_TRADEOFF) &&
		params.getFloat(PARAM_QUALITY_TRADEOFF) <= 0)
	{
		throw MLException("Quality tradeoff cannot be zero or negative.");
	}

	if (params.getBool(PARAM_ENABLE_MEDIALLY_CENTERING) &&
		params.getBool(PARAM_USE_MEDIALLY_CENTERING_TRADEOFF) &&
		params.getFloat(PARAM_MEDIALLY_CENTERING_TRADEOFF) <= 0)
	{
		throw MLException("Medially centering tradeoff cannot be zero or negative.");
	}
}

void filterSkeletonizeManual::updateBorderFlags(CMeshO& mesh, vcg::CallBackPos& callback)
{
	callback(33, "Setup - Updating border flags of selected mesh...");
	vcg::tri::UpdateFlags<CMeshO>::VertexBorderFromNone(mesh);
}

void filterSkeletonizeManual::checkSelectedMesh(CMeshO const& mesh, vcg::CallBackPos& callback)
{
	callback(66, "Setup - Checking mesh is closed...");
	for (auto& vert : mesh.vert)
	{
		if ( vert.IsB() )
		{
			throw MLException("Given model is not closed.");
		}
	}
}
