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

#include "ComputeBranches.h"

namespace curvatureSkeleton
{
	MeshBranch::MeshBranch(std::vector<int> mesh_vertices, std::vector<int> skeleton_vertices, std::vector<int> tree_vertices, Scalarm order_number, int parent_branch_index)
		: mesh_vertices(mesh_vertices), skeleton_vertices(skeleton_vertices), tree_vertices(tree_vertices), order_number(order_number), parent_branch_index(parent_branch_index) { }

	TreeBranch::TreeBranch(std::vector<int> tree_vertices, Scalarm order_number, int parent_branch_index)
		: tree_vertices(tree_vertices), order_number(order_number), parent_branch_index(parent_branch_index) { }
}
