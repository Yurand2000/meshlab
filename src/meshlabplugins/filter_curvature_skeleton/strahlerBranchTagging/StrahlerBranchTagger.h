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

#include "SkeletonTreeBuilder.h"

#include <common/ml_document/mesh_document.h>

namespace curvatureSkeleton
{

class StrahlerBranchTagger
{
public:
	typedef SkeletonTreeBuilder::SkeletonTreeNode     SkeletonTreeNode;
	typedef SkeletonTreeBuilder::SkeletonTreeBranch   SkeletonTreeBranch;
	typedef SkeletonTreeBuilder::SkeletonTreeNodes    SkeletonTreeNodes;
	typedef SkeletonTreeBuilder::SkeletonTreeBranches SkeletonTreeBranches;

	typedef CMeshO::ConstPerVertexAttributeHandle<uint> StrahlerVertexNumbers;
	typedef StrahlerVertexNumbers                       StrahlerNodeNumbers;
	typedef CMeshO::ConstPerEdgeAttributeHandle<uint>   StrahlerBranchNumbers;

public:
	static void calculateStrahlerNumbers(CMeshO& original_mesh, CMeshO& skeleton_mesh, CMeshO& tree_mesh);

	static StrahlerNodeNumbers getNodeNumbers(CMeshO const& tree_mesh);
	static StrahlerBranchNumbers getBranchNumbers(CMeshO const& tree_mesh);

	static StrahlerVertexNumbers getStrahlerNumbers(CMeshO const& mesh);
	static void strahlerNumberToQuality(CMeshO& mesh);

private:
	StrahlerBranchTagger() = delete;
};

}

#endif // FILTERCURVATURESKELETON_STRAHLER_BRANCH_TAGGER
