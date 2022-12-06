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

#include <vcg/complex/allocate.h>

#define ATTRIBUTE_MESH_TO_SKELETON_INDEX_NAME "skeleton_index"
#define ATTRIBUTE_MESH_TO_SKELETON_DISTANCE_NAME "skeleton_distance"

namespace curvatureSkeleton
{

AlgorithmSkeletonize::AlgorithmSkeletonize(
	MeshDocument&              document,
	Parameters                 parameters,
	vcg::CallBackPos&          callback_pos,
	MeshLabPluginLogger const& logger) :
		document(document),
		callback_pos(callback_pos),
		parameters(parameters),
		logger(logger)
{ }

std::map<std::string, QVariant> AlgorithmSkeletonize::apply()
{
	checkSelectedMesh();

	Skeletonizer skeletonizer = {
		Converter::toCGALMesh(getMesh()),
		parameters.skeletonizer_params
	};
	int total_iterations = skeletonize(skeletonizer);
	generateSkeleton(skeletonizer);
	addNewMeshes();

	logger.log("Algorithm terminated after " + std::to_string(total_iterations) + " iterations.");
	callback_pos(100, "Done!");
	return std::map<std::string, QVariant>();
}

void AlgorithmSkeletonize::checkSelectedMesh() const
{
	callback_pos(0, "Checking mesh is watertight...");
	auto& mesh = getMesh();

	if ( mesh.FN() == 0 )
	{
		throw MLException("Given mesh has no faces.");
	}
	else if ( !vcg::tri::Clean<CMeshO>::IsWaterTight(mesh) )
	{
		throw MLException("Given mesh is not watertight.");
	}
}

int AlgorithmSkeletonize::skeletonize(Skeletonizer& skeletonizer)
{
	int max_iters = parameters.max_iterations;
	for (int i = 1; i <= max_iters; i++)
	{
		auto curr_percent = (100.0 * i) / max_iters;
		auto curr_string  = "Computing iteration " + std::to_string(i);
		callback_pos( curr_percent, curr_string.c_str() );

		skeletonizer.computeStep();

		if (parameters.save_mesoskeletons)
			generateIntermediateMesh(skeletonizer, i);

		if (skeletonizer.hasConverged())
			return i;
	}
	return max_iters;
}

void AlgorithmSkeletonize::generateIntermediateMesh(Skeletonizer& skeletonizer, int iteration_num)
{
	auto mesoSkeleton = Converter::CGALMesoSkeletonToMesh( skeletonizer.getMesoSkeleton() );
	new_meshes.push_back(
	{
		mesoSkeleton,
		"MesoSkeleton-" + getMeshName() + QString::number(iteration_num + 1)
	});
}

void AlgorithmSkeletonize::generateSkeleton(Skeletonizer& skeletonizer)
{
	callback_pos(98, "Generating skeleton...");

	auto skeleton         = Converter::CGALSkeletonToMesh(skeletonizer.getSkeleton());
	auto mesh_to_skeleton = skeletonizer.getSkeletonVertexAssociations();

	document.mm()->setMeshModified(true);

	saveMeshToSkeletonIndex(mesh_to_skeleton);
	saveMeshToSkeletonDistance(skeleton, mesh_to_skeleton);

	new_meshes.push_back({ skeleton, "Skeleton-" + getMeshName() });
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

void AlgorithmSkeletonize::saveMeshToSkeletonIndex(MeshToSkeleton const& mesh_to_skeleton)
{
	auto& mesh     = getMesh();
	auto  iterator = vcg::tri::Allocator<CMeshO>::GetPerVertexAttribute<Scalarm>(
		mesh, ATTRIBUTE_MESH_TO_SKELETON_INDEX_NAME);

	for (auto& vertex : mesh.vert)
	{
		auto it = mesh_to_skeleton.find(vertex.Index());
		if (it != mesh_to_skeleton.end())
		{
			iterator[vertex] = it->second;
		}
		else
		{
			iterator[vertex] = -1;
		}
	}
}

void AlgorithmSkeletonize::saveMeshToSkeletonDistance(CMeshO const& skeleton, MeshToSkeleton const& mesh_to_skeleton)
{
	auto& mesh     = getMesh();
	auto  iterator = vcg::tri::Allocator<CMeshO>::GetPerVertexAttribute<Scalarm>(
		mesh, ATTRIBUTE_MESH_TO_SKELETON_DISTANCE_NAME);

	for (auto& vertex : mesh.vert)
	{
		auto it = mesh_to_skeleton.find(vertex.Index());
		if (it != mesh_to_skeleton.end())
		{
			auto& skel_vertex = skeleton.vert[it->second];
			iterator[vertex]  = (vertex.cP() - skel_vertex.cP()).Norm();
		}
	}
}

CMeshO& AlgorithmSkeletonize::getMesh() const
{
	return document.mm()->cm;
}

QString AlgorithmSkeletonize::getMeshName() const
{
	return document.mm()->label();
}

}
