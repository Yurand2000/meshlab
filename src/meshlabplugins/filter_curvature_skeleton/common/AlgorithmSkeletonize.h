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
		bool                     extend_branches;
		Scalarm                  extend_branches_angle;
	};

private:
	typedef std::vector<std::pair<CMeshO, QString>> NewMeshVector;

public:
	AlgorithmSkeletonize(
		MeshDocument&              document,
		Parameters				   parameters,
		vcg::CallBackPos&		   callback_pos,
		MeshLabPluginLogger const& logger);

	std::map<std::string, QVariant> apply();

private:
	void checkSelectedMesh() const;
	int  skeletonize(Skeletonizer& skeletonizer);
	void generateSkeleton(Skeletonizer& skeletonizer);
	void addNewMeshes();

	void generateIntermediateMesh(Skeletonizer& skeletonizer, int current_iteration);

	void saveMeshToSkeletonIndex(MeshToSkeleton const& mesh_to_skeleton);
	void saveMeshToSkeletonDistance(
		CMeshO const& skeleton, MeshToSkeleton const& mesh_to_skeleton);

	CMeshO& getMesh() const;
	QString getMeshName() const;

private:
	MeshDocument&              document;
	vcg::CallBackPos&          callback_pos;
	MeshLabPluginLogger const& logger;
	Parameters                 parameters;

	NewMeshVector                 new_meshes;
};

}

#endif //FILTERCURVATURESKELETON_ALGORITHM_SKELETONIZE
