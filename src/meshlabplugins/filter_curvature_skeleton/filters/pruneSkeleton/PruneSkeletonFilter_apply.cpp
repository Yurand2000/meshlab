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

#include "PruneSkeletonFilter.h"

#include "common/SkeletonMesh.h"

#define ATTRIBUTE_ORIGINAL_INDEX "original_index"
#define ATTRIBUTE_MESH_TO_SKELETON "skeleton_index"

namespace curvatureSkeleton
{

static void findLeaf(SkeletonMesh& skeleton, SkeletonVertex* leaf_vertex, std::vector<SkeletonVertex*>& branch_vertices, std::vector<SkeletonEdge*>& branch_edges);
static Scalarm computeLeafLenght(SkeletonMesh& skeleton, SkeletonVertex* leaf_vertex);

std::map<std::string, QVariant> PruneSkeletonFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos* cb)
{
	auto  original_mm = document.getMesh(params.getMeshId(PARAM_ORIGINAL_MESH));
	auto& original = original_mm->cm;
	auto  skeleton_mm = document.getMesh(params.getMeshId(PARAM_SKELETON_MESH));
	auto& skeleton = skeleton_mm->cm;

	auto  min_branch_lenght = params.getDynamicFloat(PARAM_MIN_EDGE_LENGHT);
	auto  remove_selected = params.getBool(PARAM_REMOVE_SELECTED_LEAFS);

	//convert to skeleton mesh
	SkeletonMesh c_skeleton;
	vcg::tri::Append<SkeletonMesh, CMeshO>::MeshCopyConst(c_skeleton, skeleton);
	vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(c_skeleton);

	//perform fix-point pruning
	bool fix_point_terminated = false;
	while (!fix_point_terminated)
	{
		//update vertex-edge adjacency
		vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(c_skeleton);

		//add map to vertex index as special attribute (needed when remapping the vertices)
		auto map = vcg::tri::Allocator<SkeletonMesh>::GetPerVertexAttribute<int>(c_skeleton, ATTRIBUTE_ORIGINAL_INDEX);
		for (auto& vert : c_skeleton.vert)
			map[vert] = vert.Index();

		//compute leafs
		using branch = std::pair<std::vector<SkeletonVertex*>, std::vector<SkeletonEdge*>>;
		std::vector<branch> leafs;
		for (auto& vert : c_skeleton.vert)
		{
			if (vcg::edge::VEDegree<SkeletonEdge>(&vert) == 1)
			{
				leafs.emplace_back();
				findLeaf(c_skeleton, &vert, leafs.back().first, leafs.back().second);
			}
		}

		//compute leafs lenghts
		std::vector<Scalarm> leaf_lenghts;
		leaf_lenghts.reserve(leafs.size());
		for (auto& leaf : leafs)
		{
			Scalarm lenght = 0;
			for (auto& edge : leaf.second)
				lenght += vcg::edge::Length(*edge);

			leaf_lenghts.emplace_back(lenght);
		}

		//select leafs by lenght (percentile) or by manual selection and find shortest leaf
		int min_leaf = -1;
		{
			Scalarm min_leaf_lenght = std::numeric_limits<Scalarm>::max();
			for (int i = 0; i < leafs.size(); i++)
			{
				bool is_leaf_selected = false;
				if (remove_selected) {
					for (auto* vert : leafs[i].first) {
						if (vert->IsS()) {
							is_leaf_selected = true;
							break;
						}
					}
				}

				bool leaf_has_to_be_removed = is_leaf_selected || leaf_lenghts[i] <= min_branch_lenght;
				bool is_the_smallest_leaf = leaf_lenghts[i] <= min_leaf_lenght;
				if (leaf_has_to_be_removed && is_the_smallest_leaf)
				{
					min_leaf = i;
					min_leaf_lenght = leaf_lenghts[i];
				}
			}
		}

		//fix-point termination
		if (min_leaf < 0)
		{
			fix_point_terminated = true;
		}
		//remove shortest leaf
		else
		{
			//remove edges and vertices, remap indices
			std::unordered_map<int, int> indices_remap;
			for (auto& vert : c_skeleton.vert)
				indices_remap.emplace(map[vert], map[vert]);

			{
				vcg::tri::UnMarkAll(skeleton);
				std::vector<SkeletonEdge*> star;

				auto& vertices = leafs[min_leaf].first;
				auto& edges = leafs[min_leaf].second;

				auto leaf_end_index = map[vertices.back()];
				for (int i = 0; i < vertices.size() - 1; i++)
				{
					auto* vert = vertices[i];
					indices_remap[ map[vert] ] = leaf_end_index;
					vcg::tri::Allocator<SkeletonMesh>::DeleteVertex(c_skeleton, *vert);
				}

				for (auto* edge : edges)
					vcg::tri::Allocator<SkeletonMesh>::DeleteEdge(c_skeleton, *edge);
			}

			//compact vector
			vcg::tri::Allocator<SkeletonMesh>::CompactEveryVector(c_skeleton);

			//remap indices to new vertices indices
			std::unordered_map<int, int> inverse_remap;
			for (auto& vert : c_skeleton.vert)
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

	//save updated skeleton
	vcg::tri::Append<CMeshO, SkeletonMesh>::MeshCopy(skeleton, c_skeleton);

	return {};
}

void findLeaf(SkeletonMesh& skeleton, SkeletonVertex* leaf_vertex, std::vector<SkeletonVertex*>& branch_vertices, std::vector<SkeletonEdge*>& branch_edges)
{
	assert(vcg::edge::VEDegree<SkeletonEdge>(leaf_vertex) == 1);

	vcg::tri::UnMarkAll(skeleton);

	SkeletonVertex *old = nullptr, *current = leaf_vertex;
	std::vector<SkeletonEdge*> star;
	vcg::edge::VEStarVE(current, star);

	while (star.size() <= 2 && current != old)
	{
		old = current;
		vcg::tri::Mark(skeleton, current);
		branch_vertices.emplace_back(current);

		for (auto* adj_edge : star) {
			auto* adj = adj_edge->V( adj_edge->V(0) != current ? 0 : 1 );

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

Scalarm computeLeafLenght(SkeletonMesh& skeleton, SkeletonVertex* leaf_vertex)
{
	assert( vcg::edge::VEDegree<SkeletonEdge>(leaf_vertex) == 1 );

	Scalarm lenght = 0;
	vcg::tri::UnMarkAll(skeleton);

	SkeletonVertex *old = nullptr, *current = leaf_vertex;
	std::vector<SkeletonVertex*> star;
	do
	{
		old = current;
		vcg::tri::Mark(skeleton, current);
		vcg::edge::VVStarVE(current, star);
		for (auto* adj : star) {
			if (!vcg::tri::IsMarked(skeleton, adj))
			{
				lenght += vcg::Distance(current->cP(), adj->cP());
				current = adj;
				break;
			}
		}

	} while (star.size() <= 2 && current != old);

	return lenght;
}

}
