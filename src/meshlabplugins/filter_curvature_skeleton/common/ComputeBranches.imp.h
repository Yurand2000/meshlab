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

#include <vcg/complex/complex.h>

namespace curvatureSkeleton
{
	namespace detail
	{
		struct TreeBranch
		{
			std::vector<int> tree_vertices;
			Scalarm order_number;
			int parent_branch_index;
		};

		template<typename MESH> static std::vector<TreeBranch> computeTreeBranches(
			MESH& tree, int tree_root_index,
			std::string const& input_attribute_name,
			std::function<bool(Scalarm const&, Scalarm const&)> input_attribute_compare_function
		);

		template<typename MESH> static std::vector<SkeletonBranch> computeSkeletonBranches(
			MESH& skeleton, MESH& tree, std::vector<detail::TreeBranch> tree_branches
		);

		template<typename MESH> static void computeOriginalBranches(
			MESH& original, std::vector<SkeletonBranch>& branches,
			std::string const& adjacency_attribute_name
		);

		template<typename MESH> static void extractMesh(
			MESH& original, MESH& branch,
			int branch_index, std::vector<SkeletonBranch> const& branches_data
		);

		template<typename MESH> static void selectFromBranch(
			MESH& original, int branch_index, std::vector<SkeletonBranch> const& branches_data
		);

		template<typename MESH> static void selectBiggestConnectedComponent(MESH& branch);
		template<typename MESH> static void removeNotSelectedAndCompact(MESH& branch);
	}

	template<typename MESH>
	std::vector<SkeletonBranch> ComputeBranches<MESH>::compute(
		MESH& original, MESH& skeleton, MESH& tree, int tree_root_index,
		std::string const& input_attribute_name,
		std::function<bool(Scalarm const&, Scalarm const&)> input_attribute_compare_function,
		std::string const& adjacency_attribute_name
	) {
		auto tree_branches = detail::computeTreeBranches(tree, tree_root_index, input_attribute_name, input_attribute_compare_function);
		auto branches = detail::computeSkeletonBranches(skeleton, tree, tree_branches);
		detail::computeOriginalBranches(original, branches, adjacency_attribute_name);

		return branches;
	}

	template<typename MESH>
	std::vector<detail::TreeBranch> detail::computeTreeBranches(
		MESH& tree, int tree_root_index,
		std::string const& attribute,
		std::function<bool(Scalarm const&, Scalarm const&)> less
	) {
		using Branch = detail::TreeBranch;
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

		auto order_numbers = Allocator::GetPerVertexAttribute<Scalarm>(tree, attribute);

		std::vector<Branch> branches;
		std::stack<std::pair<SkeletonVertex*, int>> frontier;

		branches.push_back( {
			{ tree_root_index },
			order_numbers[tree_root_index],
			-1
		} );
		frontier.push( { &converted_tree.vert[tree_root_index], 0 } );

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

					if (less(adj_order_number, order_number))
					{
						branches.push_back( {
							{ node->Index(), adj_index },
							adj_order_number,
							branch
						} );
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
	std::vector<SkeletonBranch> detail::computeSkeletonBranches(
		MESH& skeleton, MESH& tree, std::vector<detail::TreeBranch> tree_branches
	) {
		using ToSkeletonMeshAppend = vcg::tri::Append<SkeletonMesh, MESH>;

		std::vector<SkeletonBranch> branches;
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
		for (auto& branch : branches) {

			for (int i = 0; i < branch.tree_vertices.size() - 1; i++)
			{
				auto* start = &converted_skeleton.vert[Utils<SkeletonMesh>::getVertexIndexInMesh(tree.vert[branch.tree_vertices[i]].cP(), converted_skeleton)];
				auto* end   = &converted_skeleton.vert[Utils<SkeletonMesh>::getVertexIndexInMesh(tree.vert[branch.tree_vertices[i+1]].cP(), converted_skeleton)];

				auto skel_vertices = EdgeMeshUtils<SkeletonMesh>::getVerticesInBetween(converted_skeleton, start, end);
				for (auto* vertex : skel_vertices) {
					branch.skeleton_vertices.push_back(vertex->Index());
				}
			}
		}

		return branches;
	}

	template<typename MESH>
	void detail::computeOriginalBranches(
		MESH& original, std::vector<SkeletonBranch>& branches,
		std::string const& adjacency_attribute_name
	) {
		using Allocator = vcg::tri::Allocator<MESH>;

		auto adj_attribute = Allocator::template GetPerVertexAttribute<Scalarm>(original, adjacency_attribute_name);

		for (auto& branch : branches) {
			std::unordered_set<int> indices;
			std::copy(branch.skeleton_vertices.begin(), branch.skeleton_vertices.end(), std::inserter(indices, indices.end()));

			for (auto& vertex : original.vert)
			{
				if (indices.count( adj_attribute[vertex] ) > 0) {
					branch.mesh_vertices.push_back( vertex.Index() );
				}
			}
		}
	}


	template<typename MESH>
	std::vector<MESH> ComputeBranches<MESH>::extractBranches(
		MESH& original,
		std::vector<SkeletonBranch>& branches_data
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
		std::vector<SkeletonBranch> const& branches_data
	) {
		//select and copy the given branch
		detail::selectFromBranch(original, branch_index, branches_data);
		vcg::tri::UpdateSelection<MESH>::VertexClear(original);

		//copy branch into mesh
		vcg::tri::Append<MESH, MESH>::MeshCopy(branch, original, true);
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
		std::vector<SkeletonBranch> const& branches_data
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
