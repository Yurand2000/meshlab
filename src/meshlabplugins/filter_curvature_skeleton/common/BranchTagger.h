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

#ifndef FILTERCURVATURESKELETON_BRANCH_TAGGER
#define FILTERCURVATURESKELETON_BRANCH_TAGGER

#include <common/ml_document/cmesh.h>
#include <vector>

namespace curvatureSkeleton
{

template<typename MESH>
class BranchTagger
{
public:
	typedef vcg::Color4b Color;

	static void generateTreeMesh(MESH& tree, MESH const& skeleton, int root_index, Scalarm percentile);
	static void treeScalarAttributeToSkeleton(MESH& skeleton, MESH const& tree, std::string attribute_name, bool prioritize_small_values = true);
	static void skeletonScalarAttributeToOriginalMesh(MESH& mesh, MESH& skeleton, std::string attribute_name);

	static std::vector<Color> generateColorsFromAttribute(MESH const& mesh, std::string attribute_name);
	static void colorizeByAttribute(MESH& mesh, std::vector<Color> const& colors, std::string attribute_name);

private:
	~BranchTagger() = delete;
};

}

#include "BranchTagger.imp.h"

#endif // FILTERCURVATURESKELETON_BRANCH_TAGGER
