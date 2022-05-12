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

#ifndef FILTERCURVATURESKELETON_STRAHLER_BRANCH_TAGGER
#define FILTERCURVATURESKELETON_STRAHLER_BRANCH_TAGGER

#include "SkeletonMesh.h"
#include <common/ml_document/cmesh.h>

#define ATTRIBUTE_STRAHLER_NUMBER "strahler_number"

namespace curvatureSkeleton
{

class StrahlerBranchTagger
{
public:
	static void generateTreeMesh(SkeletonMesh& tree, CMeshO const& skeleton, int root_index, Scalarm percentile);
	static void strahlerNumbersToSkeleton(CMeshO& skeleton, SkeletonMesh const& tree, int root_index);
	static void strahlerNumbersToOriginalMesh(CMeshO& mesh, CMeshO& skeleton);

private:
	~StrahlerBranchTagger() = delete;
};

}

#endif // FILTERCURVATURESKELETON_STRAHLER_BRANCH_TAGGER
