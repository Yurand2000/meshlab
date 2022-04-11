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

#include "AlgorithmSkeletonize.h"
#include "SkeletonizeFilter.h"
#include "common/additionalAttributeNames.h"
#include <vcg/complex/allocate.h>

namespace curvatureSkeleton
{

AlgorithmSkeletonize::AlgorithmSkeletonize(
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

std::map<std::string, QVariant> AlgorithmSkeletonize::apply(
	int  max_iterations,
	bool generate_intermediate_meshes,
	bool skeleton_distance_in_mesh_quality)
{
	checkSelectedMesh();
	skeletonizer         = std::make_unique<Skeletonizer>(mesh, parameters);
	int total_iterations = skeletonize(max_iterations, generate_intermediate_meshes);
	generateSkeleton(skeleton_distance_in_mesh_quality);
	addNewMeshes();
	

	logger.log("Algorithm terminated after " + std::to_string(total_iterations) + " iterations.");
	callback_pos(100, "Done!");
	return std::map<std::string, QVariant>();
}

void AlgorithmSkeletonize::checkSelectedMesh() const
{
	callback_pos(0, "Checking mesh is watertight...");
	if (!vcg::tri::Clean<CMeshO>::IsWaterTight(mesh)) {
		throw MLException("Given mesh is not watertight.");
	}
}


int AlgorithmSkeletonize::skeletonize(int max_iters, bool gen_intermediate_meshes)
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


bool AlgorithmSkeletonize::computeIteration()
{
	skeletonizer->computeStep();
	return skeletonizer->hasConverged();
}

void AlgorithmSkeletonize::generateIntermediateMesh(int iteration_num)
{
	auto mesoSkeleton = skeletonizer->getMesoSkeleton();
	new_meshes.push_back(
		std::make_pair(
			mesoSkeleton,
			mesh_name + "-mesoSkeleton" + QString::number(iteration_num + 1)
	));
}

void AlgorithmSkeletonize::generateSkeleton(bool skeleton_distance_in_mesh_quality)
{
	callback_pos(98, "Generating skeleton...");

	auto skeleton         = skeletonizer->getSkeleton();
	auto mesh_to_skeleton = skeletonizer->getSkeletonVertexAssociations();

	document.mm()->setMeshModified(true);

	saveMeshToSkeletonIndex(mesh_to_skeleton);
	saveMeshToSkeletonDistance(skeleton_distance_in_mesh_quality, skeleton, mesh_to_skeleton);

	new_meshes.push_back(std::make_pair(skeleton, mesh_name + "-skeleton"));
}

void AlgorithmSkeletonize::addNewMeshes()
{
	callback_pos(99, "Adding new meshes...");
	for (auto pair : new_meshes)
	{
		auto mesh = document.addNewMesh(pair.first, pair.second, false);
		mesh->clearDataMask(MeshModel::MM_VERTQUALITY);
	}
}

void AlgorithmSkeletonize::saveMeshToSkeletonIndex(Skeletonizer::MeshToSkeletonVertices const& mesh_to_skeleton)
{
	auto iterator = vcg::tri::Allocator<CMeshO>::GetPerVertexAttribute<uint>(
		mesh, ATTRIBUTE_MESH_TO_SKELETON_INDEX_NAME);

	for (uint i = 0; i < mesh.vert.size(); i++)
	{
		auto it = mesh_to_skeleton.find(i);
		if (it != mesh_to_skeleton.end())
		{
			uint skel_index = it->second;
			iterator[i]     = skel_index;
		}
	}
}

void AlgorithmSkeletonize::saveMeshToSkeletonDistance(
	bool										skeleton_distance_in_mesh_quality,
	CMeshO const&                               skeleton,
	Skeletonizer::MeshToSkeletonVertices const& mesh_to_skeleton)
{
	auto iterator = vcg::tri::Allocator<CMeshO>::GetPerVertexAttribute<Scalarm>(
		mesh, ATTRIBUTE_MESH_TO_SKELETON_DISTANCE_NAME);

	if (skeleton_distance_in_mesh_quality)
		document.mm()->updateDataMask(MeshModel::MM_VERTQUALITY);

	for (uint i = 0; i < mesh.vert.size(); i++)
	{
		auto it = mesh_to_skeleton.find(i);
		if (it != mesh_to_skeleton.end())
		{
			auto& vertex    = mesh.vert[i];
			auto& skel_vertex = skeleton.vert[it->second];
			iterator[i]       = (vertex.cP() - skel_vertex.cP()).Norm();

			if (skeleton_distance_in_mesh_quality)
				vertex.Q() = iterator[i];
		}
	}
}

}
