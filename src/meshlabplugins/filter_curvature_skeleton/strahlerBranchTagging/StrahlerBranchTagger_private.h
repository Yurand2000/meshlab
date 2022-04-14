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

#ifndef FILTERCURVATURESKELETON_STRAHLER_BRANCH_TAGGER_PRIVATE
#define FILTERCURVATURESKELETON_STRAHLER_BRANCH_TAGGER_PRIVATE

#include "StrahlerBranchTagger.h"

#define ATTRIBUTE_TREE_NODE_STRAHLER_NUMBER "tree_node_strahler_number"
#define ATTRIBUTE_TREE_BRANCH_STRAHLER_NUMBER "tree_branch_strahler_number"
#define ATTRIBUTE_VERTEX_STRAHLER_NUMBER "strahler_number"

namespace curvatureSkeleton
{

class StrahlerNumberCalculator
{
public:
	StrahlerNumberCalculator(CMeshO& tree_mesh);

	void compute();

	static StrahlerBranchTagger::StrahlerNodeNumbers     getNodeNumbers(CMeshO const& tree_mesh);
	static StrahlerBranchTagger::StrahlerBranchNumbers getBranchNumbers(CMeshO const& tree_mesh);

private:
	CMeshO& tree;
};

class SaveStrahlerNumber
{
public:
	SaveStrahlerNumber(CMeshO& original_mesh, CMeshO& skeleton, CMeshO const& tree);

	void saveNumbers();

	static void strahlerNumberToQuality(CMeshO& mesh);

private:
	CMeshO& original;
	CMeshO& skeleton;
	CMeshO const& tree;
};

}

#endif //FILTERCURVATURESKELETON_STRAHLER_BRANCH_TAGGER_PRIVATE
