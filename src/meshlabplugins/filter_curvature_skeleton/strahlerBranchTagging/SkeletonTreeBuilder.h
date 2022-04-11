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

#include <vector>
#include <common/ml_document/mesh_document.h>
#include <vcg/complex/allocate.h>
#include <boost/optional.hpp>

namespace curvatureSkeleton
{

class SkeletonTreeBuilder
{
public:
	struct SkeletonTreeNode
	{
		uint                  skeleton_vertex;
		boost::optional<uint> previous_branch;
		std::vector<uint>     next_branches;

		bool isRoot();
	};

	struct SkeletonTreeBranch
	{
		std::vector<uint> skeleton_vertices;
		uint              previous_node;
		uint              next_node;
	};

	typedef CMeshO::ConstPerVertexAttributeHandle<SkeletonTreeNode> SkeletonTreeNodes;
	typedef CMeshO::ConstPerEdgeAttributeHandle<SkeletonTreeBranch> SkeletonTreeBranches;

public:
	static void checkSkeletonTree(CMeshO const& skeleton);

	static CMeshO generateTree(CMeshO const& skeleton, uint root_index);

	static SkeletonTreeNodes    getTreeNodes(CMeshO const& tree);
	static SkeletonTreeBranches getTreeBranches(CMeshO const& tree);

private:
	SkeletonTreeBuilder() = delete;
};

}

#endif //FILTERCURVATURESKELETON_SKELETON_TREE_BUILDER
