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

#include "PruneSkeleton.h"

#define ATTRIBUTE_ORIGINAL_INDEX "original_index"
#define ATTRIBUTE_MESH_TO_SKELETON "skeleton_index"

namespace curvatureSkeleton
{
using Branch = std::tuple<std::vector<SkeletonVertex*>, std::vector<SkeletonEdge*>, Scalarm>;
static std::vector<Branch> computeBranches(SkeletonMesh& skeleton);
static bool tryGetBranchToPrune(std::vector<Branch> const& branches, Scalarm min_branch_lenght, bool remove_selected, int& out_prune_index);
static void findLeaf(SkeletonMesh& skeleton, SkeletonVertex* leaf_vertex, std::vector<SkeletonVertex*>& branch_vertices, std::vector<SkeletonEdge*>& branch_edges);

void PruneSkeleton::pruneSkeleton(CMeshO& original, SkeletonMesh& skeleton, Scalarm min_branch_length, bool remove_selected) {
	vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(skeleton);
	while (true)
	{
		//update vertex-edge adjacency
		vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(skeleton);

		//add map to vertex index as special attribute (needed when remapping the vertices)
		auto map = vcg::tri::Allocator<SkeletonMesh>::GetPerVertexAttribute<int>(skeleton, ATTRIBUTE_ORIGINAL_INDEX);
		for (auto& vert : skeleton.vert)
			map[vert] = vert.Index();

		//compute branches and their lenghts
		auto branches = computeBranches(skeleton);

		int branch_to_prune;
		//select leafs by lenght (percentile) or by manual selection and find the shortest leaf to be pruned
		if (!tryGetBranchToPrune(branches, min_branch_length, remove_selected, branch_to_prune)) {
			break;
		}

		//remove edges and vertices, remap indices
		std::unordered_map<int, int> indices_remap;
		for (auto& vert : skeleton.vert)
			indices_remap.emplace(map[vert], map[vert]);

		{
			vcg::tri::UnMarkAll(skeleton);
			std::vector<SkeletonEdge*> star;

			auto& vertices = std::get<0>(branches[branch_to_prune]);
			auto& edges = std::get<1>(branches[branch_to_prune]);

			auto leaf_end_index = map[vertices.back()];
			for (int i = 0; i < vertices.size() - 1; i++)
			{
				auto* vert = vertices[i];
				indices_remap[map[vert]] = leaf_end_index;
				vcg::tri::Allocator<SkeletonMesh>::DeleteVertex(skeleton, *vert);
			}

			for (auto* edge : edges)
				vcg::tri::Allocator<SkeletonMesh>::DeleteEdge(skeleton, *edge);
		}

		//compact vector
		vcg::tri::Allocator<SkeletonMesh>::CompactEveryVector(skeleton);

		//remap indices to new vertices indices
		std::unordered_map<int, int> inverse_remap;
		for (auto& vert : skeleton.vert)
			inverse_remap.emplace(map[vert], vert.Index());

		//remap original mesh indices to new vertices indices
		auto mesh_to_skeleton = vcg::tri::Allocator<CMeshO>::GetPerVertexAttribute<Scalarm>(original, ATTRIBUTE_MESH_TO_SKELETON);
		for (auto& vert : original.vert)
		{
			auto original_skeleton_index = mesh_to_skeleton[vert];
			auto remap_to_collapsing_point = indices_remap[original_skeleton_index];
			auto remap_to_compacted_vector = inverse_remap[remap_to_collapsing_point];
			mesh_to_skeleton[vert] = remap_to_compacted_vector;
		}
	}
}

std::vector<Branch> computeBranches(SkeletonMesh& skeleton) {
	std::vector<Branch> branches;
	for (auto& vert : skeleton.vert)
	{
		if (vcg::edge::VEDegree<SkeletonEdge>(&vert) == 1)
		{
			branches.emplace_back();
			findLeaf(skeleton, &vert, std::get<0>(branches.back()), std::get<1>(branches.back()));
		}
	}

	for (auto& branch : branches) {
		Scalarm length = 0;
		for (auto& edge : std::get<1>(branch))
			length += vcg::edge::Length(*edge);

		std::get<2>(branch) = length;
	}

	return branches;
}

bool tryGetBranchToPrune(std::vector<Branch> const& branches, Scalarm min_branch_lenght, bool remove_selected, int& out_prune_index) {
	int min_branch = -1;
	Scalarm curr_min_branch_lenght = std::numeric_limits<Scalarm>::max();
	for (int i = 0; i < branches.size(); i++)
	{
		auto branch_length = std::get<2>(branches[i]);

		bool is_leaf_selected = false;
		if (remove_selected) {
			for (auto* vert : std::get<0>(branches[i])) {
				if (vert->IsS()) {
					is_leaf_selected = true;
					break;
				}
			}
		}

		bool leaf_has_to_be_removed = is_leaf_selected || branch_length <= min_branch_lenght;
		bool is_the_smallest_leaf = branch_length <= curr_min_branch_lenght;
		if (leaf_has_to_be_removed && is_the_smallest_leaf)
		{
			min_branch = i;
			curr_min_branch_lenght = branch_length;
		}
	}

	if (min_branch >= 0) {
		out_prune_index = min_branch;
		return true;
	}
	else {
		out_prune_index = -1;
		return false;
	}
}

void findLeaf(SkeletonMesh& skeleton, SkeletonVertex* leaf_vertex, std::vector<SkeletonVertex*>& branch_vertices, std::vector<SkeletonEdge*>& branch_edges)
{
	assert(vcg::edge::VEDegree<SkeletonEdge>(leaf_vertex) == 1);

	vcg::tri::UnMarkAll(skeleton);

	SkeletonVertex* old = nullptr, * current = leaf_vertex;
	std::vector<SkeletonEdge*> star;
	vcg::edge::VEStarVE(current, star);

	while (star.size() <= 2 && current != old)
	{
		old = current;
		vcg::tri::Mark(skeleton, current);
		branch_vertices.emplace_back(current);

		for (auto* adj_edge : star) {
			auto* adj = adj_edge->V(adj_edge->V(0) != current ? 0 : 1);

			if (!vcg::tri::IsMarked(skeleton, adj))
			{
				branch_edges.emplace_back(adj_edge);
				current = adj;
				break;
			}
		}

		vcg::edge::VEStarVE(current, star);
	}

	branch_vertices.emplace_back(current);
}
}
