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

#include "algorithm_skeletonize.h"
#include "skeletonize_manual.h"

algorithmSkeletonize::algorithmSkeletonize(
	MeshDocument&              document,
	Parameters&                parameters,
	vcg::CallBackPos&          callback_pos,
	MeshLabPluginLogger const& logger) :
		document(document),
		callback_pos(callback_pos),
		mesh(document.mm()->cm),
		parameters(parameters),
		skeletonizer(),
		logger(logger)
{
	mesh_name    = QFileInfo(document.mm()->fullName()).baseName();
	new_meshes   = NewMeshVector();
}

algorithmSkeletonize::~algorithmSkeletonize()
{
	if (skeletonizer != nullptr) {
		delete skeletonizer;
	}
}



std::map<std::string, QVariant> algorithmSkeletonize::apply(
	int  max_iterations,
	bool generate_intermediate_meshes,
	bool skeleton_distance_in_mesh_quality)
{
	checkSelectedMesh();
	skeletonizer = new Skeletonizer(mesh, parameters);
	int total_iterations = skeletonize(max_iterations, generate_intermediate_meshes);
	generateSkeleton(skeleton_distance_in_mesh_quality);
	addNewMeshes();
	

	logger.log("Algorithm terminated after " + std::to_string(total_iterations) + " iterations.");
	callback_pos(100, "Done!");
	return std::map<std::string, QVariant>();
}

void algorithmSkeletonize::checkSelectedMesh() const
{
	callback_pos(0, "Checking mesh is watertight...");
	if (!vcg::tri::Clean<CMeshO>::IsWaterTight(mesh)) {
		throw MLException("Given mesh is not watertight.");
	}
}


int algorithmSkeletonize::skeletonize(int max_iters, bool gen_intermediate_meshes)
{
	int  i         = 0;
	bool converged = false;
	for (i = 0; i < max_iters && !converged; i++)
	{
		callback_pos(
			((100.0 * (i+1)) / max_iters),
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
	skeletonizer->computeStep();
	return skeletonizer->hasConverged();
}

void algorithmSkeletonize::generateIntermediateMesh(int iteration_num)
{
	auto mesoSkeleton = skeletonizer->getMesoSkeleton();
	new_meshes.push_back(
		std::make_pair(
			mesoSkeleton,
			mesh_name + "-mesoSkeleton" + QString::number(iteration_num + 1)
	));
}

void algorithmSkeletonize::generateSkeleton(bool skeleton_distance_in_mesh_quality)
{
	callback_pos(98, "Generating skeleton...");

	auto skeleton = skeletonizer->getSkeleton();
	new_meshes.push_back(std::make_pair(skeleton, mesh_name + "-skeleton"));

	if (skeleton_distance_in_mesh_quality)
	{
		auto mesh_to_skeleton = skeletonizer->getSkeletonVertexAssociations();

		document.mm()->updateDataMask(MeshModel::MM_VERTQUALITY);
		document.mm()->setMeshModified(true);
		for (uint i = 0; i < mesh.vert.size(); i++)
		{
			auto& vertex = mesh.vert[i];

			auto& skel_vertex = skeleton.vert[mesh_to_skeleton.at(i)];
			vertex.Q() = (vertex.cP() - skel_vertex.cP()).Norm();
		}
	}
}

void algorithmSkeletonize::addNewMeshes()
{
	callback_pos(99, "Adding new meshes...");
	for (auto pair : new_meshes) {
		document.addNewMesh(pair.first, pair.second, false);
	}
}
