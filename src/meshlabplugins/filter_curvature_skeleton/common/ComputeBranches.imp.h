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

#ifndef FILTERCURVATURESKELETON_COMPUTE_BRANCHES_IMP
#define FILTERCURVATURESKELETON_COMPUTE_BRANCHES_IMP

#include "common/SkeletonMesh.h"
#include "common/EdgeMeshUtils.h"
#include "common/Utils.h"

#include <unordered_set>
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/clean.h>

namespace curvatureSkeleton
{
	namespace detail
	{
		template<typename MESH> static std::vector<TreeBranch> computeTreeBranches(
			MESH& tree, int tree_root_index,
			std::string const& input_attribute_name,
			std::function<bool(Scalarm const&, Scalarm const&)> input_attribute_compare_function
		);

		template<typename MESH> static std::vector<MeshBranch> computeSkeletonBranches(
			MESH& skeleton, MESH& tree, std::vector<TreeBranch>& tree_branches,
			std::string const& input_attribute_name
		);

		template<typename MESH> static void computeOriginalBranches(
			MESH& original, std::vector<MeshBranch>& branches,
			std::string const& input_attribute_name,
			std::string const& adjacency_attribute_name
		);

		template<typename MESH> static void extractMesh(
			MESH& original, MESH& branch,
			int branch_index, std::vector<MeshBranch> const& branches_data
		);

		template<typename MESH> static void selectFromBranch(
			MESH& original, int branch_index, std::vector<MeshBranch> const& branches_data
		);

		template<typename MESH> static void selectBiggestConnectedComponent(MESH& branch);
		template<typename MESH> static void removeNotSelectedAndCompact(MESH& branch);
	}

	template<typename MESH>
	std::vector<MeshBranch> ComputeBranches<MESH>::compute(
		MESH& original, MESH& skeleton, MESH& tree, int tree_root_index,
		std::string const& input_attribute_name,
		std::function<bool(Scalarm const&, Scalarm const&)> input_attribute_compare_function,
		std::string const& adjacency_attribute_name
	) {
		auto tree_branches = detail::computeTreeBranches(tree, tree_root_index, input_attribute_name, input_attribute_compare_function);
		auto branches = detail::computeSkeletonBranches(skeleton, tree, tree_branches, input_attribute_name);
		detail::computeOriginalBranches(original, branches, input_attribute_name, adjacency_attribute_name);

		return branches;
	}

	template<typename MESH>
	std::vector<TreeBranch> detail::computeTreeBranches(
		MESH& tree, int tree_root_index,
		std::string const& attribute,
		std::function<bool(Scalarm const&, Scalarm const&)> less
	) {
		using Branch = TreeBranch;
		using Allocator = vcg::tri::Allocator<MESH>;
		using ToSkeletonMeshAppend = vcg::tri::Append<SkeletonMesh, MESH>;
		using FromSkeletonMeshAppend = vcg::tri::Append<MESH, SkeletonMesh>;
		using SkeletonTopology = vcg::tri::UpdateTopology<SkeletonMesh>;

		//convert tree to SkeletonMesh
		SkeletonMesh converted_tree;
		vcg::tri::Allocator<SkeletonMesh>::AddPerVertexAttribute<Scalarm>(converted_tree, attribute);
		ToSkeletonMeshAppend::MeshCopyConst(converted_tree, tree);
		SkeletonTopology::VertexEdge(converted_tree);
		vcg::tri::InitVertexIMark(converted_tree);
		vcg::tri::InitEdgeIMark(converted_tree);

		auto order_numbers = Allocator::template GetPerVertexAttribute<Scalarm>(tree, attribute);

		std::vector<Branch> branches;
		std::stack<std::pair<SkeletonVertex*, int>> frontier;

		branches.push_back( Branch(
			{ tree_root_index },
			order_numbers[tree_root_index],
			-1
		) );
		frontier.push( { &converted_tree.vert[tree_root_index], 0 } );

		//breadth-first search to extract branches
		vcg::tri::UnMarkAll(converted_tree);
		do
		{
			auto* node = frontier.top().first;
			auto  branch = frontier.top().second;
			vcg::tri::Mark(converted_tree, node);
			frontier.pop();

			auto order_number = order_numbers[node->Index()];

			std::vector<SkeletonVertex*> verts;
			vcg::edge::VVStarVE(node, verts);
			for (auto* adj : verts)
			{
				if (!vcg::tri::IsMarked(converted_tree, adj))
				{
					auto adj_index = adj->Index();
					auto adj_order_number = order_numbers[adj_index];

					//if the order number of the adjacent vertex is (less) than the current branch's order number then
					//this vertex is not part of the branch, but it is instead a children.
					//otherwise it is part of the current branch.
					if (less(adj_order_number, order_number))
					{
						branches.push_back( Branch(
							{ node->Index(), adj_index },
							adj_order_number,
							branch
						) );
						frontier.push({ adj, branches.size() - 1 });
					}
					else
					{
						branches[branch].tree_vertices.push_back(adj_index);
						frontier.push({ adj, branch });
					}
				}
			}
		} while (!frontier.empty());

		return branches;
	}

	template<typename MESH>
	std::vector<MeshBranch> detail::computeSkeletonBranches(
		MESH& skeleton, MESH& tree, std::vector<TreeBranch>& tree_branches,
		std::string const& input_attribute_name
	) {
		using ToSkeletonMeshAppend = vcg::tri::Append<SkeletonMesh, MESH>;

		auto order_numbers = vcg::tri::Allocator<MESH>::template GetPerVertexAttribute<Scalarm>(skeleton, input_attribute_name);

		std::vector<MeshBranch> branches;
		branches.reserve(tree_branches.size());
		//copy tree branches
		for (auto& branch : tree_branches) {
			branches.emplace_back();
			branches.back().tree_vertices.swap(branch.tree_vertices);
			branches.back().order_number = branch.order_number;
			branches.back().parent_branch_index = branch.parent_branch_index;
		}

		//convert skeleton to SkeletonMesh
		SkeletonMesh converted_skeleton;
		ToSkeletonMeshAppend::MeshCopyConst(converted_skeleton, skeleton);
		vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(converted_skeleton);
		vcg::tri::InitVertexIMark(converted_skeleton);

		//get skeleton paths
		auto& mesh = converted_skeleton;
		for (auto& branch : branches)
		{
			//corresponding points from tree to skeleton
			for (int i = 0; i < branch.tree_vertices.size() - 1; i++)
			{
				auto* start = Utils<SkeletonMesh>::getVertexInMesh(tree.vert[branch.tree_vertices[i]].cP(), mesh);
				auto* end = Utils<SkeletonMesh>::getVertexInMesh(tree.vert[branch.tree_vertices[i + 1]].cP(), mesh);

				auto path = EdgeMeshUtils<SkeletonMesh>::getVerticesPath(mesh, start, end);
				for (auto it = path.rbegin(); it != path.rend(); it++) {
					auto* vertex = *it;

					//keep only the vertices with the same order number
					if (order_numbers[vertex->Index()] == branch.order_number) {
						branch.skeleton_vertices.push_back(vertex->Index());
					}
					else {
						break;
					}
				}
			}

			if (branch.skeleton_vertices.size() > 2)
			{
				//fill the frontier for the flooding operation
				auto& indices = branch.skeleton_vertices;
				std::queue<SkeletonVertex*> frontier;
				vcg::tri::UnMarkAll(mesh);
				vcg::tri::Mark(mesh, &mesh.vert[indices.front()]);
				vcg::tri::Mark(mesh, &mesh.vert[indices.back()]);
				for (size_t i = 1; i < indices.size() - 1; i++) {
					frontier.push(&mesh.vert[indices[i]]);
					vcg::tri::Mark(mesh, &mesh.vert[indices[i]]);
				}

				//add vertices recursively by flooding
				std::vector<SkeletonVertex*> star;
				do {
					auto* node = frontier.front(); frontier.pop();
					vcg::tri::Mark(mesh, node);

					vcg::edge::VVStarVE(node, star);
					for (auto* adj : star) {
						if (!vcg::tri::IsMarked(mesh, adj))
						{
							if (order_numbers[adj->Index()] == branch.order_number) {
								branch.skeleton_vertices.push_back(adj->Index());
								frontier.push(adj);
							}
							else {
								vcg::tri::Mark(mesh, adj);
							}
						}
					}
				} while (!frontier.empty());
			}
		}

		return branches;
	}

	template<typename MESH>
	void detail::computeOriginalBranches(
		MESH& original, std::vector<MeshBranch>& branches,
		std::string const& input_attribute_name,
		std::string const& adjacency_attribute_name
	) {
		using Allocator = vcg::tri::Allocator<MESH>;

		auto order_attribute = Allocator::template GetPerVertexAttribute<Scalarm>(original, input_attribute_name);
		auto adj_attribute = Allocator::template GetPerVertexAttribute<Scalarm>(original, adjacency_attribute_name);

		for (auto& branch : branches) {
			std::unordered_set<int> indices;
			std::copy(branch.skeleton_vertices.begin(), branch.skeleton_vertices.end(), std::inserter(indices, indices.end()));

			for (auto& vertex : original.vert)
			{
				if (indices.count( adj_attribute[vertex] ) > 0 && order_attribute[vertex] == branch.order_number) {
					branch.mesh_vertices.push_back( vertex.Index() );
				}
			}
		}
	}


	template<typename MESH>
	std::vector<MESH> ComputeBranches<MESH>::extractBranches(
		MESH& original,
		std::vector<MeshBranch>& branches_data
	) {
		std::vector<MESH> branches;
		branches.reserve(branches_data.size());
		for (int i = 0; i < branches_data.size(); i++)
		{
			branches.emplace_back();
			detail::extractMesh(original, branches.back(), i, branches_data);
		}
		vcg::tri::UpdateSelection<MESH>::Clear(original);

		return branches;
	}

	template<typename MESH>
	void detail::extractMesh(
		MESH& original,
		MESH& branch,
		int branch_index,
		std::vector<MeshBranch> const& branches_data
	) {
		//select the given branch
		detail::selectFromBranch(original, branch_index, branches_data);

		//copy branch into mesh
		vcg::tri::Append<MESH, MESH>::MeshCopyConst(branch, original, true);
		vcg::tri::UpdateSelection<MESH>::Clear(branch);

		//get only the biggest connected component of the branch
		branch.face.EnableFFAdjacency();
		vcg::tri::UpdateTopology<MESH>::FaceFace(branch);
		detail::selectBiggestConnectedComponent(branch);
		branch.face.DisableFFAdjacency();

		//clean branch
		detail::removeNotSelectedAndCompact(branch);
		vcg::tri::UpdateSelection<MESH>::Clear(branch);
	}

	template<typename MESH>
	void detail::selectFromBranch(
		MESH& original, int branch_index,
		std::vector<MeshBranch> const& branches_data
	) {
		vcg::tri::UpdateSelection<MESH>::VertexClear(original);

		std::queue<int> branch_nums;
		branch_nums.push(branch_index);
		do
		{
			auto curr = branch_nums.front(); branch_nums.pop();
			auto& branch = branches_data[curr];

			for (auto index : branch.mesh_vertices) {
				original.vert[index].SetS();
			}

			for (int j = 0; j < branches_data.size(); j++) {
				if (branches_data[j].parent_branch_index == curr) {
					branch_nums.push(j);
				}
			}
		} while (!branch_nums.empty());
		vcg::tri::UpdateSelection<MESH>::FaceFromVertexStrict(original);
	}

	template<typename MESH>
	void detail::selectBiggestConnectedComponent(MESH& branch)
	{
		vcg::tri::RequireFFAdjacency(branch);
		if ( branch.face.empty() ) return;

		std::vector<std::pair<int, CFaceO*>> conn_comps;
		vcg::tri::Clean<MESH>::ConnectedComponents(branch, conn_comps);

		//get biggest connected component
		int index = 0;
		{
			int size = 0;
			for (int j = 0; j < conn_comps.size(); j++)
			{
				if (conn_comps[j].first > size) {
					size = conn_comps[j].first;
					index = j;
				}
			}
		}

		//select biggest connected component
		vcg::tri::UpdateSelection<MESH>::FaceClear(branch);
		conn_comps[index].second->SetS();
		vcg::tri::UpdateSelection<MESH>::FaceConnectedFF(branch);
		vcg::tri::UpdateSelection<MESH>::VertexFromEdgeLoose(branch);
		vcg::tri::UpdateSelection<MESH>::VertexFromFaceLoose(branch);
	}

	template<typename MESH>
	void detail::removeNotSelectedAndCompact(MESH& branch)
	{
		for (auto& face : branch.face) {
			if (!face.IsD() && !face.IsS()) face.SetD();
		}
		for (auto& vert : branch.vert) {
			if (!vert.IsD() && !vert.IsS()) vert.SetD();
		}

		vcg::tri::Allocator<MESH>::CompactFaceVector(branch);
		vcg::tri::Allocator<MESH>::CompactVertexVector(branch);
	}
}

#endif // FILTERCURVATURESKELETON_COMPUTE_BRANCHES_IMP
