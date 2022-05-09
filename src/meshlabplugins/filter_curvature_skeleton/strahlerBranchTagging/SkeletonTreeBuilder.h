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

#ifndef FILTERCURVATURESKELETON_SKELETON_TREE_BUILDER
#define FILTERCURVATURESKELETON_SKELETON_TREE_BUILDER

#include "common/SkeletonMesh.h"
#include <common/ml_document/mesh_document.h>

namespace curvatureSkeleton
{

Scalarm calculateEdgeLengthPercentile(SkeletonMesh const& mesh, Scalarm percentile);

void generateTreeMesh(
	SkeletonMesh& tree,
	CMeshO const& skeleton,
	int           root_index,
	Scalarm       percentile);

}

#endif //FILTERCURVATURESKELETON_SKELETON_TREE_BUILDER
