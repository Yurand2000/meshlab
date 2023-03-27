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

#ifndef FILTERCURVATURESKELETON_ALGORITHM_SKELETONIZE
#define FILTERCURVATURESKELETON_ALGORITHM_SKELETONIZE

#include <common/plugins/interfaces/filter_plugin.h>
#include "cgalAdapter/CGALMeshConverter.h"
#include "cgalAdapter/CGALMeshSkeletonizer.h"

namespace curvatureSkeleton
{

class AlgorithmSkeletonize
{
public:
	typedef CGALMeshConverter<CMeshO>            Converter;
	typedef CGALMeshSkeletonizer                 Skeletonizer;
	typedef Skeletonizer::MeshToSkeletonVertices MeshToSkeleton;

	struct Parameters
	{
		Skeletonizer::Parameters skeletonizer_params;
		int                      max_iterations;
		bool                     save_mesoskeletons;
	};

private:
	typedef std::vector<CMeshO> NewMeshVector;

public:
	AlgorithmSkeletonize(
		vcg::CallBackPos&		   callback_pos,
		MeshLabPluginLogger const& logger);

	CMeshO skeletonize(CMeshO& mesh, Parameters parameters, bool log_output = true, NewMeshVector* intermediate_meshes = nullptr);

private:
	void checkMesh(CMeshO& mesh) const;
	int  skeletonize(Skeletonizer& skeletonizer, Parameters parameters, NewMeshVector* intermediate_meshes);
	CMeshO generateSkeleton(CMeshO& mesh, Skeletonizer& skeletonizer);

	void generateIntermediateMesh(Skeletonizer& skeletonizer, int current_iteration, NewMeshVector* intermediate_meshes);

	void saveMeshToSkeletonIndex(CMeshO& mesh, MeshToSkeleton const& mesh_to_skeleton);
	void saveMeshToSkeletonDistance(
		CMeshO& mesh, CMeshO const& skeleton, MeshToSkeleton const& mesh_to_skeleton);

private:
	vcg::CallBackPos&          callback_pos;
	MeshLabPluginLogger const& logger;
};

}

#endif //FILTERCURVATURESKELETON_ALGORITHM_SKELETONIZE
