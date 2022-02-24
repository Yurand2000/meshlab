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
#include "algorithm_skeletonize.h"

#define _USE_MATH_DEFINES
#include <cmath>
#include <string>

algorithmSkeletonize::Parameters getSkeletonizerParameters(RichParameterList const& params);
int getIterationCount(RichParameterList const& params);
bool getGenerateIntermediateMeshes(RichParameterList const& params);
bool getSaveSkeletonDistance(RichParameterList const& params);

std::map<std::string, QVariant> filterSkeletonizeManual::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos* callback)
{
	try {
		auto& selected_mesh = document.mm()->cm;
		checkParameters(params, *callback);

		auto skel_params   = getSkeletonizerParameters(params);
		int  iterations    = getIterationCount(params);
		bool gen_meshes    = getGenerateIntermediateMeshes(params);
		bool skel_distance = getSaveSkeletonDistance(params);
		return algorithmSkeletonize(document, skel_params, *callback, plugin)
			.apply(iterations, gen_meshes, skel_distance);
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
	callback(0, "Setup: Checking Parameters...");
	if (params.getInt(PARAM_MAX_ITERATIONS) < 1)
	{
		throw MLException("Number of iterations cannot be less than 1.");
	}

	if (params.getFloat(PARAM_DELTA_AREA_TERMINATION) <= 0) {
		throw MLException("Delta area convergence cannot be zero or negative.");
	}

	if (params.getFloat(PARAM_MAX_ANGLE) < (90)) {
		throw MLException("Max triangle angle cannot be less than 90 degrees.");
	}
	if (params.getFloat(PARAM_MAX_ANGLE) >= (180)) {
		throw MLException("Max triangle angle cannot be greater than 180 degrees.");
	}

	if (params.getFloat(PARAM_MIN_EDGE_LENGTH) <= 0)
	{
		throw MLException("Min edge length cannot be zero or negative.");
	}

	if (params.getFloat(PARAM_QUALITY_TRADEOFF) <= 0)
	{
		throw MLException("Quality tradeoff cannot be zero or negative.");
	}

	if (params.getBool(PARAM_ENABLE_MEDIALLY_CENTERING) &&
		params.getFloat(PARAM_MEDIALLY_CENTERING_TRADEOFF) <= 0)
	{
		throw MLException("Medially centering tradeoff cannot be zero or negative.");
	}
}

algorithmSkeletonize::Parameters getSkeletonizerParameters(RichParameterList const& params)
{
	algorithmSkeletonize::Parameters skel_params = {};

	skel_params.delta_area_threshold   = params.getFloat(PARAM_DELTA_AREA_TERMINATION);
	skel_params.max_triangle_angle     = params.getFloat(PARAM_MAX_ANGLE);
	skel_params.min_edge_length        = params.getFloat(PARAM_MIN_EDGE_LENGTH);
	skel_params.quality_speed_tradeoff = params.getFloat(PARAM_QUALITY_TRADEOFF);

	if (params.getBool(PARAM_ENABLE_MEDIALLY_CENTERING))
		skel_params.medially_centering_speed_tradeoff =
			params.getFloat(PARAM_MEDIALLY_CENTERING_TRADEOFF);

	return skel_params;
}

int getIterationCount(RichParameterList const& params)
{
	return params.getInt(PARAM_MAX_ITERATIONS);
}

bool getGenerateIntermediateMeshes(RichParameterList const& params)
{
	return params.getBool(PARAM_GENERATE_INTERMEDIATE_MESHES);
}

bool getSaveSkeletonDistance(RichParameterList const& params)
{
	return params.getBool(PARAM_SAVE_SKELETAL_DISTANCE_TO_MESH_QUALITY);
}
