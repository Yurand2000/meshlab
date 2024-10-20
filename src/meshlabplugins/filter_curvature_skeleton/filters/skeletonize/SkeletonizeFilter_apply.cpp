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

#include "SkeletonizeFilter.h"

#include <string>
#include <vcg/math/base.h>

#include "cgalAdapter/CGALMeshConverter.h"
#include "common/AlgorithmSkeletonize.h"
#include "common/SkeletonMesh.h"
#include "common/PruneSkeleton.h"

namespace curvatureSkeleton
{

typedef CGALMeshConverter<CMeshO>            Converter;
typedef CGALMeshSkeletonizer                 Skeletonizer;
typedef Skeletonizer::MeshToSkeletonVertices MeshToSkeletonVertices;

static AlgorithmSkeletonize::Parameters getParameters(RichParameterList const& params);

std::map<std::string, QVariant> SkeletonizeFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& rich_params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos* callback)
{
	try {
		auto& selected_mesh = document.mm()->cm;
		auto mesh_name = document.mm()->label();
		checkParameters(rich_params, *callback);


		//generate skeleton
		auto params = getParameters(rich_params);
		auto skeleton = AlgorithmSkeletonize(*callback, plugin)
			.skeletonize(selected_mesh, params, true);

		document.mm()->setMeshModified(true);

		//save skeleton mesh
		auto skeleton_mesh = document.addNewMesh(QString(), QString("Skeleton - %1").arg(mesh_name), false);
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopyConst(skeleton_mesh->cm, skeleton);
		skeleton_mesh->clearDataMask(MeshModel::MM_VERTQUALITY);
		skeleton_mesh->updateBoxAndNormals();

		//log number of branches
			//convert to skeleton mesh
		SkeletonMesh c_skeleton;
		vcg::tri::Append<SkeletonMesh, CMeshO>::MeshCopyConst(c_skeleton, skeleton);
		auto num_branches = PruneSkeleton::getNumBranches(c_skeleton);

		plugin.log( QString("%2 - Number of branches: %1").arg(num_branches).arg(mesh_name).toStdString() );

		return {
			{"num_branches", QVariant(num_branches)}
		};
	}
	catch (MLException e) {
		throw e;
	}
	catch (std::exception e) {
		throw MLException(QString("Unhandled exception: ") + QString(e.what()));
	}
}



void SkeletonizeFilter::checkParameters(RichParameterList const& params, vcg::CallBackPos& callback)
{
	callback(0, "Setup: Checking Parameters...");
	if (params.getInt(PARAM_MAX_ITERATIONS) < 1)
	{
		throw MLException("Number of iterations cannot be less than 1.");
	}

	if (params.getAbsPerc(PARAM_MIN_EDGE_LENGTH) <= 0)
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

static Skeletonizer::Parameters getSkeletonizerParameters(RichParameterList const& params);

static AlgorithmSkeletonize::Parameters getParameters(RichParameterList const& rich_params)
{
	AlgorithmSkeletonize::Parameters params;
	params.skeletonizer_params   = getSkeletonizerParameters(rich_params);

	params.max_iterations        = rich_params.getInt(PARAM_MAX_ITERATIONS);

	return params;
}

Skeletonizer::Parameters getSkeletonizerParameters(RichParameterList const& params)
{
	Skeletonizer::Parameters skel_params = {};

	skel_params.delta_area_threshold   = params.getDynamicFloat(PARAM_DELTA_AREA_TERMINATION);
	skel_params.max_triangle_angle     = params.getDynamicFloat(PARAM_MAX_ANGLE);
	skel_params.min_edge_length        = params.getAbsPerc(PARAM_MIN_EDGE_LENGTH);
	skel_params.quality_speed_tradeoff = params.getFloat(PARAM_QUALITY_TRADEOFF);

	if (params.getBool(PARAM_ENABLE_MEDIALLY_CENTERING))
		skel_params.medially_centering_speed_tradeoff =
			params.getFloat(PARAM_MEDIALLY_CENTERING_TRADEOFF);

	return skel_params;
}

}
