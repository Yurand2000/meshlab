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

#ifndef FILTERCURVATURESKELETON_COMPUTE_BRANCHES
#define FILTERCURVATURESKELETON_COMPUTE_BRANCHES

#include <common/ml_document/cmesh.h>
#include <vector>

namespace curvatureSkeleton
{

struct MeshBranch
{
	std::vector<int> tree_vertices, skeleton_vertices, mesh_vertices;
	Scalarm order_number;
	int parent_branch_index;

	MeshBranch() = default;
	MeshBranch(std::vector<int>, std::vector<int>, std::vector<int>, Scalarm, int);
};

struct TreeBranch
{
	std::vector<int> tree_vertices;
	Scalarm order_number;
	int parent_branch_index;

	TreeBranch() = default;
	TreeBranch(std::vector<int>, Scalarm, int);
};

template<typename MESH>
class ComputeBranches
{
public:
	static std::vector<MeshBranch> compute(
		MESH& original, MESH& skeleton,
		MESH& tree, int tree_root_index,
		std::string const& input_attribute_name,
		std::function<bool(Scalarm const&, Scalarm const&)> input_attribute_compare_function,
		std::string const& adjacency_attribute_name);

	static std::vector<MESH> extractBranches(
		MESH& original, std::vector<MeshBranch>& branches
	);

private:
	~ComputeBranches() = delete;
};

}

#include "ComputeBranches.imp.h"

#endif // FILTERCURVATURESKELETON_COMPUTE_BRANCHES
