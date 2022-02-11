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

#include "skeletonize.h"
#include "../filter_curvature_skeleton_param_names.h"

algorithmSkeletonize::algorithmSkeletonize(
	MeshDocument&              document,
	RichParameterList const&   parameters,
	vcg::CallBackPos&          callback_pos,
	MeshLabPluginLogger const& logger) :
		document(document),
		parameters(parameters),
	    callback_pos(callback_pos),
		mesh(document.mm()->cm),
		skeletonizer(mesh, getSkeletonizerParameters()),
		logger(logger)
{
	mesh_name    = QFileInfo(document.mm()->fullName()).baseName();
	new_meshes   = NewMeshVector();
}

algorithmSkeletonize::SkelParams algorithmSkeletonize::getSkeletonizerParameters()
{
	SkelParams skel_params = {};

	if (parameters.getBool(PARAM_USE_MAX_ANGLE))
		skel_params.max_triangle_angle = parameters.getFloat(PARAM_MAX_ANGLE);

	if (parameters.getBool(PARAM_USE_MIN_EDGE_LENGTH))
		skel_params.min_edge_length = parameters.getFloat(PARAM_MIN_EDGE_LENGTH);

	if (parameters.getBool(PARAM_USE_QUALITY_TRADEOFF))
		skel_params.quality_speed_tradeoff = parameters.getFloat(PARAM_QUALITY_TRADEOFF);

	if (parameters.getBool(PARAM_ENABLE_MEDIALLY_CENTERING) &&
		parameters.getBool(PARAM_USE_MEDIALLY_CENTERING_TRADEOFF))
		skel_params.medially_centered_speed_tradeoff =
			parameters.getFloat(PARAM_MEDIALLY_CENTERING_TRADEOFF);

	if (parameters.getBool(PARAM_USE_DELTA_AREA_TERMINATION))
		skel_params.delta_area_threshold = parameters.getFloat(PARAM_DELTA_AREA_TERMINATION);
	return skel_params;
}

algorithmSkeletonize::~algorithmSkeletonize() { }



std::map<std::string, QVariant> algorithmSkeletonize::apply()
{
	int  max_iterations               = getIterationCount();
	bool generate_intermediate_meshes = getGenerateIntermediateMeshes();

	int total_iterations = skeletonize(max_iterations, generate_intermediate_meshes);
	generateSkeleton();

	callback_pos(99, "Adding new meshes...");
	for (auto pair : new_meshes) {
		document.addNewMesh(pair.first, pair.second);
	}

	logger.log("Algorithm terminated after " + std::to_string(total_iterations) + " iterations.");

	callback_pos(100, "Done!");
	return std::map<std::string, QVariant>();
}

int algorithmSkeletonize::getIterationCount()
{
	if (parameters.getBool(PARAM_SINGLE_ITERATION))
		return 1;
	else
		return parameters.getInt(PARAM_MAX_ITERATIONS);
}

bool algorithmSkeletonize::getGenerateIntermediateMeshes()
{
	return parameters.getBool(PARAM_GENERATE_INTERMEDIATE_MESHES);
}



int algorithmSkeletonize::skeletonize(int max_iters, bool gen_intermediate_meshes)
{
	int  i         = 0;
	bool converged = false;
	for (i = 0; i < max_iters && !converged; i++)
	{
		callback_pos(
			((double(i)) / max_iters),
			("Computing iteration " + std::to_string(i + 1) + " of " + std::to_string(max_iters)).c_str()
		);

		converged = computeIteration();
		if (gen_intermediate_meshes)
			generateIntermediateMesh(i);
	}
	return i;
}


bool algorithmSkeletonize::computeIteration()
{
	skeletonizer.computeStep();
	return skeletonizer.hasConverged();
}

void algorithmSkeletonize::generateIntermediateMesh(int iteration_num)
{
	auto mesoSkeleton = skeletonizer.getMesoSkeleton();
	new_meshes.push_back(
		std::make_pair(
			mesoSkeleton,
			mesh_name + "-mesoSkeleton" + QString::number(iteration_num + 1)
	));
}



void algorithmSkeletonize::generateSkeleton()
{
	callback_pos(98, "Generating skeleton...");

	auto skeleton = skeletonizer.getSkeleton();
	new_meshes.push_back(std::make_pair(skeleton, mesh_name + "-skeleton"));
}
