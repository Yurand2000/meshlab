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
#include "../cgal_adapter/mesh_skeletonizer.h"

class algorithmSkeletonize
{
	typedef CGalAdapter::MeshSkeletonizer           Skeletonizer;
	typedef CGalAdapter::MeshSkeletonizerParameters SkelParams;
	typedef std::vector<std::pair<CMeshO, QString>> NewMeshVector;

public:
	algorithmSkeletonize(
		MeshDocument&              document,
		RichParameterList const&   parameters,
		vcg::CallBackPos&		   callback_pos,
		MeshLabPluginLogger const& logger);
	~algorithmSkeletonize();

	std::map<std::string, QVariant> apply();

private:
	SkelParams algorithmSkeletonize::getSkeletonizerParameters();
	int		   getIterationCount();
	bool       getGenerateIntermediateMeshes();

	int        skeletonize(int max_iterations, bool generate_intermediate_meshes);
	bool	   computeIteration();
	void       generateIntermediateMesh(int current_iteration);
	void       generateSkeleton();

private:
	MeshDocument& document;
	RichParameterList const& parameters;
	vcg::CallBackPos& callback_pos;
	MeshLabPluginLogger const& logger;
	CMeshO& mesh;

	QString       mesh_name;
	Skeletonizer  skeletonizer;
	NewMeshVector new_meshes;
};

#endif
