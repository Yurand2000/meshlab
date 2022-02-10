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

#include "filter_curvature_skeleton.h"
#include "filter_curvature_skeleton_param_names.h"
#include "cgal_adapter/mesh_skeletonizer.h"

#include <string>

/**
 * @brief The Real Core Function doing the actual mesh processing.
 * @param action
 * @param md: an object containing all the meshes and rasters of MeshLab
 * @param par: the set of parameters of each filter
 * @param cb: callback object to tell MeshLab the percentage of execution of the filter
 * @return true if the filter has been applied correctly, false otherwise
 */
std::map<std::string, QVariant> FilterCurvatureSkeleton::applyFilter(const QAction* action, const RichParameterList& parameters, MeshDocument& document, unsigned int&, vcg::CallBackPos* cb)
{
	switch (ID(action))
	{
	case CURVATURE_SKELETON:
	{
		try
		{
			auto& selected_mesh = document.mm()->cm;

			checkParameters(parameters, *cb);
			updateBorderFlags(selected_mesh, *cb);
			checkSelectedMesh(selected_mesh, *cb);
			cb(100, "Setup - Done.");
			return applyAlgorithm(selected_mesh, document, parameters, *cb);
		}
		catch (MLException e)
		{
			throw e;
		}
		catch(std::exception e)
		{
			throw MLException( QString("Unhandled exception: ") + QString(e.what()) );
		}
	}
	default:
		wrongActionCalled(action);
	}
	return std::map<std::string, QVariant>();
}


void FilterCurvatureSkeleton::checkParameters(RichParameterList const& params, vcg::CallBackPos& callback)
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

void FilterCurvatureSkeleton::updateBorderFlags(CMeshO& mesh, vcg::CallBackPos& callback)
{
	callback(33, "Setup - Updating border flags of selected mesh...");
	vcg::tri::UpdateFlags<CMeshO>::VertexBorderFromNone(mesh);
}

void FilterCurvatureSkeleton::checkSelectedMesh(CMeshO const& mesh, vcg::CallBackPos& callback)
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

typedef std::vector<std::pair<CMeshO, QString>> NewMeshVector;

CGalAdapter::MeshSkeletonizerParameters getSkeletonizerParameters(RichParameterList const&);
int  getIterationCount(RichParameterList const&);
bool getGenerateIntermediateMeshes(RichParameterList const&);

std::map<std::string, QVariant> FilterCurvatureSkeleton::applyAlgorithm(CMeshO const& mesh, MeshDocument& document, RichParameterList const& parameters, vcg::CallBackPos& callback_pos)
{
	auto model_name   = document.mm()->shortName();
	auto skeletonizer = CGalAdapter::MeshSkeletonizer( mesh, getSkeletonizerParameters(parameters) );

	int max_iterations = getIterationCount(parameters);
	bool generate_intermediate_meshes = getGenerateIntermediateMeshes(parameters);
	
	int i = 0;
	bool converged = false;
	NewMeshVector new_meshes = {};
	for (i = 0; i < max_iterations && !converged; i++)
	{
		callback_pos(((double(i)) / max_iterations), ("Computing iteration " + std::to_string(i+1) + " of " + std::to_string(max_iterations) + " - Skeletonization Step...").c_str() );
		skeletonizer.computeStep();
		converged = skeletonizer.hasConverged();

		if (generate_intermediate_meshes)
		{
			callback_pos(((double(i)) / max_iterations), ("Computing iteration " + std::to_string(i+1) + " of " + std::to_string(max_iterations) + " - Generating Intermediate Mesh...").c_str() );
			auto mesoSkeleton = skeletonizer.getMesoSkeleton();
			new_meshes.push_back(std::make_pair(mesoSkeleton, model_name + "-mesoSkeleton" + QString::number(i+1)));
		}
	}

	callback_pos(98, "Generating skeleton...");
	auto skeleton = skeletonizer.getSkeleton();
	new_meshes.push_back(std::make_pair(skeleton, model_name + "-skeleton"));

	callback_pos(99, "Adding new meshes...");
	for (auto pair : new_meshes)
	{
		document.addNewMesh(pair.first, pair.second);
	}

	log("Algorithm terminated after " + std::to_string(i) + " iterations.");

	callback_pos(100, "Done!");
	return std::map<std::string, QVariant>();
}

CGalAdapter::MeshSkeletonizerParameters getSkeletonizerParameters(RichParameterList const& params)
{
	CGalAdapter::MeshSkeletonizerParameters skel_params = {};
	if (params.getBool(PARAM_USE_MAX_ANGLE))
		skel_params.max_triangle_angle = params.getFloat(PARAM_MAX_ANGLE);
	if (params.getBool(PARAM_USE_MIN_EDGE_LENGTH))
		skel_params.min_edge_length = params.getFloat(PARAM_MIN_EDGE_LENGTH);
	if (params.getBool(PARAM_USE_QUALITY_TRADEOFF))
		skel_params.quality_speed_tradeoff = params.getFloat(PARAM_QUALITY_TRADEOFF);
	if (params.getBool(PARAM_ENABLE_MEDIALLY_CENTERING) && params.getBool(PARAM_USE_MEDIALLY_CENTERING_TRADEOFF))
		skel_params.medially_centered_speed_tradeoff = params.getFloat(PARAM_MEDIALLY_CENTERING_TRADEOFF);
	if (params.getBool(PARAM_USE_DELTA_AREA_TERMINATION))
		skel_params.delta_area_threshold = params.getFloat(PARAM_DELTA_AREA_TERMINATION);
	return skel_params;
}

int getIterationCount(RichParameterList const& params)
{
	if (params.getBool(PARAM_SINGLE_ITERATION))
		return 1;
	else
		return params.getInt(PARAM_MAX_ITERATIONS);
}

bool getGenerateIntermediateMeshes(RichParameterList const& params)
{
	return params.getBool(PARAM_GENERATE_INTERMEDIATE_MESHES);
}

MESHLAB_PLUGIN_NAME_EXPORTER(FilterCurvatureSkeleton)
