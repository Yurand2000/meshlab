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

#include "TreeSegmentationFilter.h"

#include "common/SkeletonMesh.h"
#include "common/SimplifySkeleton.h"
#include "common/EdgeMeshUtils.h"
#include <vcg/space/colormap.h>

#define ATTRIBUTE_ORIGINAL_INDEX "original_index"
#define ATTRIBUTE_MESH_TO_SKELETON "skeleton_index"
#define ATTRIBUTE_HACK_ORDER_NUMBER "hack_order"

namespace curvatureSkeleton
{

static std::vector<SkeletonVertex*> getNonMarkedSubtree(SkeletonMesh& mesh, SkeletonVertex* vertex);
static std::pair<SkeletonVertex*, SkeletonVertex*> getClosestNonTwoConnectedNeighbors(SkeletonMesh& mesh, SkeletonVertex* vertex);

std::map<std::string, QVariant> TreeSegmentationFilter::applyFilter(
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

	auto root_selected = params.getBool(PARAM_SELECTED_ROOT);
	auto swap_selected_branches = params.getBool(PARAM_SELECTED_BRANCHES);
	auto map_to_color = params.getBool(PARAM_MAP_TO_COLOR);
	auto save_graph = params.getBool(PARAM_SAVE_GRAPH);
	auto facetag_id = params.getString(PARAM_FACE_TAG_ID);

	//find the root (selected or lowest point on Y axis)
	int c_root_index = 0;
	if (root_selected && skeleton.svn != 1)
	{
		throw MLException("Cannot define a root point: zero or more than one vertex is selected on the skeleton.");
	}
	else if (root_selected)
	{
		for (auto& vert : skeleton.vert) {
			if (vert.IsS()) {
				c_root_index = vert.Index();
				break;
			}
		}
	}
	else
	{
		Scalarm lowest_y = std::numeric_limits<Scalarm>::max();
		for (auto& vert : skeleton.vert)
		{
			if ( vert.cP().Y() < lowest_y )
			{
				lowest_y = vert.cP().Y();
				c_root_index = vert.Index();
			}
		}
	}
	
	//generate straightened graph (remove two connected vertices)
	SkeletonMesh c_skeleton, graph;
	vcg::tri::Append<SkeletonMesh, CMeshO>::MeshCopyConst(c_skeleton, skeleton);
	vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(c_skeleton);
	vcg::tri::InitVertexIMark(c_skeleton);
	vcg::tri::InitEdgeIMark(c_skeleton);

	vcg::tri::Append<SkeletonMesh, SkeletonMesh>::MeshCopyConst(graph, c_skeleton);
	vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(graph);

	if (!SimplifySkeleton<SkeletonMesh>::isMeshConnected(graph))
		throw MLException("Given graph mesh must be connected.");

	SimplifySkeleton<SkeletonMesh>::collapseTwoConnectedVertices(graph, c_root_index);
	vcg::tri::Allocator<SkeletonMesh>::CompactEveryVector(graph);
	vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(graph);
	vcg::tri::InitVertexIMark(graph);
	vcg::tri::InitEdgeIMark(graph);

	//find root index on graph
	int root_index = 0;
	for (auto& vert : graph.vert)
	{
		if (vert.cP() == skeleton.vert[c_root_index].cP())
			root_index = vert.Index();
	}

	//compute hack order
	vcg::tri::Stat<SkeletonMesh>::ComputeHackOrderNumbers(graph, root_index, ATTRIBUTE_HACK_ORDER_NUMBER);
	auto order_number = vcg::tri::Allocator<SkeletonMesh>::GetPerVertexAttribute<Scalarm>(graph, ATTRIBUTE_HACK_ORDER_NUMBER);

	//swap selected branches
	if (swap_selected_branches)
	{
		//list of tree braches to prioritize. Each point pair corresponds to a graph's edge and its sub-graph.
		std::set<std::pair<vcg::Point3<Scalarm>, vcg::Point3<Scalarm>>> swap_branches;

		//from the selections on the mesh extract the points on the skeleton.
		std::unordered_set<SkeletonVertex*> selected_vertices;

		auto mesh_to_skeleton = vcg::tri::Allocator<CMeshO>::GetPerVertexAttribute<Scalarm>(original, ATTRIBUTE_MESH_TO_SKELETON);
		for (auto& face : original.face)
		{
			if (!face.IsD() && face.IsS()) {
				for (int i = 0; i < 3; i++) {
					auto skeleton_index = mesh_to_skeleton[face.V(i)];
					selected_vertices.emplace(&c_skeleton.vert[skeleton_index]);
				}
			}
		}

		//from the points on the skeleton extract the graph edges.
		for (auto* vert : selected_vertices)
		{
			if (vcg::edge::VEDegree<SkeletonEdge>(vert) == 2)
			{
				auto neighbors = getClosestNonTwoConnectedNeighbors(c_skeleton, vert);

				//avoid duplicates by ensuring any arbitrary order.
				if (neighbors.first > neighbors.second)
					std::swap(neighbors.first, neighbors.second);

				swap_branches.emplace( neighbors.first->cP(), neighbors.second->cP() );
			}
		}

		//starting from the root, at each intersection eval if and how many swap branches requestes there are.
		vcg::tri::UnMarkAll(graph);
		std::queue<SkeletonVertex*> frontier;

		//build starting frontier
		std::vector<SkeletonVertex*> star;
		vcg::edge::VVStarVE(&graph.vert[root_index], star);
		vcg::tri::Mark(graph, &graph.vert[root_index]);

		for (auto* vert : star)
			frontier.push(vert);

		//foreach node in the frontier evaluate its star of adjacent nodes
		while ( !frontier.empty() )
		{
			auto* node = frontier.front(); frontier.pop();
			vcg::tri::Mark(graph, node);
			vcg::edge::VVStarVE(node, star);

			int selected = 0;
			int selected_order = -1, min_order = std::numeric_limits<int>::max();
			SkeletonVertex* selected_node = nullptr;
			for (auto* vert : star)
			{
				if ( !vcg::tri::IsMarked(graph, vert) )
				{
					frontier.push(vert);

					//process order number
					auto vert_order = order_number[vert];
					if (vert_order < min_order)
						min_order = vert_order;

					//count how many selected
					for (auto& edge : swap_branches)
					{
						if ((edge.first == node->cP() && edge.second == vert->cP()) ||
							(edge.first == vert->cP() && edge.second == node->cP())
						) {
							selected += 1;
							selected_order = vert_order;
							selected_node = vert;
						}
					}
				}
			}

			//if too many selected or the selected has already higher priority (lower order number), do nothing.
			if (selected != 1 || selected_order == min_order)
				continue;

			for (auto* child_node : star)
			{
				if ( vcg::tri::IsMarked(graph, child_node) ) continue;

				//if selected, increase priority of the subtree (reduce order number)
				if (selected_node == child_node)
				{
					for (auto* vert : getNonMarkedSubtree(graph, child_node))
					{
						order_number[vert] -= 1;
					}
				}
				//if the order is the minimum, this branch has to be lowered in priority (increase order number)
				else if (order_number[child_node] == min_order)
				{
					for (auto* vert : getNonMarkedSubtree(graph, child_node))
					{
						order_number[vert] += 1;
					}
				}
			}
		}
	}

	//initialize facetag attribute
	auto facetag = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(original, facetag_id.toStdString());
	for (auto& face : original.face)
		facetag[face] = -1;

	std::unordered_map<int, int> face_tag_to_hack_map;

	//tree segmentation
	{
		//foreach leaf
		for (auto& vert : graph.vert)
		{
			//find leafs only
			if ( vcg::edge::VEDegree<SkeletonEdge>(&vert) != 1 || vert.Index() == root_index )
				continue;

			//find the branch (go towards the root)
			SkeletonVertex *tip_vertex = &vert, *base_vertex = nullptr;
			bool exclude_base = false;

			auto current_hack_number = order_number[tip_vertex];
			std::vector<SkeletonEdge*> star;

			SkeletonVertex *old = nullptr, *current = tip_vertex;
			vcg::edge::VEStarVE(current, star);
			vcg::tri::UnMarkAll(graph);
			while ( current != old )
			{
				old = current;
				vcg::tri::Mark(graph, current);

				for (auto* adj_edge : star) {
					auto* adj = adj_edge->V(adj_edge->V(0) != current ? 0 : 1);

					if ( !vcg::tri::IsMarked(graph, adj) )
					{
						if (current_hack_number == order_number[adj])
						{
							current = adj;
						}
						else if (current_hack_number - 1 == order_number[adj])
						{
							exclude_base = true;
							base_vertex = adj;
						}
					}
				}

				vcg::edge::VEStarVE(current, star);
			}

			if (base_vertex == nullptr)
				base_vertex = current;

			//find skeleton vertices (path between the tip and base vertices)
			SkeletonVertex *c_base_vertex = nullptr, *c_tip_vertex = nullptr;
			for (auto& vert : c_skeleton.vert)
			{
				if ( vert.cP() == base_vertex->cP() )
					c_base_vertex = &vert;
				if ( vert.cP() == tip_vertex->cP() )
					c_tip_vertex = &vert;
			}

			if (c_base_vertex == nullptr || c_tip_vertex == nullptr)
				throw MLException("Generic error [1]");

			auto c_vertices = EdgeMeshUtils<SkeletonMesh>::getVerticesPath(c_skeleton, c_tip_vertex, c_base_vertex);
			if (exclude_base)
				c_vertices.pop_back();

			//foreach skeleton vertex find original mesh vertices/faces
			auto mesh_to_skeleton = vcg::tri::Allocator<CMeshO>::GetPerVertexAttribute<Scalarm>(original, ATTRIBUTE_MESH_TO_SKELETON);

			std::unordered_set<int> c_indices;
			for (auto* vert : c_vertices)
				c_indices.insert(vert->Index());

			std::vector<CVertexO*> o_vertices;
			for (auto& vert : original.vert)
			{
				if ( c_indices.count( mesh_to_skeleton[vert] ) > 0 )
					o_vertices.emplace_back(&vert);
			}

			//tag faces of the branch
			vcg::tri::SelectionStack<CMeshO> selection(original);
			selection.push();
			vcg::tri::UpdateSelection<CMeshO>::Clear(original);
			auto facetag = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(original, facetag_id.toStdString());
			for (auto* vert : o_vertices)
				vert->SetS();

			vcg::tri::UpdateSelection<CMeshO>::FaceFromVertexStrict(original);
			for (auto& face : original.face)
			{
				if (!face.IsD() && face.IsS())
					facetag[face] = tip_vertex->Index();
			}

			face_tag_to_hack_map.emplace(tip_vertex->Index(), current_hack_number);

			selection.pop();
		}
	}

	//save graph as mesh
	if (save_graph)
	{
		auto graph_mm = document.addNewMesh(QString(), QString("Graph - %1").arg( original_mm->label() ), false);
		vcg::tri::Allocator<CMeshO>::AddPerVertexAttribute<Scalarm>(graph_mm->cm, ATTRIBUTE_HACK_ORDER_NUMBER);
		vcg::tri::Append<CMeshO, SkeletonMesh>::MeshCopyConst(graph_mm->cm, graph);
	}

	//save per face hack number attribute
	auto faceorder = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(original, ATTRIBUTE_HACK_ORDER_NUMBER);
	for (auto& face : original.face)
		faceorder[face] = -1;

	for (auto& face : original.face)
		faceorder[face] = face_tag_to_hack_map[ facetag[face] ];

	//map to color
	if (map_to_color)
	{
		auto num_colors = face_tag_to_hack_map.size();
		original.face.EnableColor();
		original_mm->updateDataMask(MeshModel::MeshElement::MM_FACECOLOR);

		for (auto& face : original.face)
		{
			if (face.IsD())
				continue;

			auto face_id = facetag[face];
			if (face_id >= 0)
			{
				auto hack_number = face_tag_to_hack_map[face_id];
				auto color = vcg::GetColorMapping((hack_number - 1) % 6, 0, 5, vcg::Plasma);
				face.C() = color;
			}
			else
			{
				face.C() = vcg::Color4b(100, 100, 100, 255);
			}
		}
	}

	return {};
}

std::vector<SkeletonVertex*> getNonMarkedSubtree(SkeletonMesh& mesh, SkeletonVertex* vertex)
{
	std::vector<SkeletonVertex*> subtree, star;
	std::unordered_set<SkeletonVertex*> local_mark;
	std::queue<SkeletonVertex*> frontier;
	frontier.push(vertex);

	for (auto& vert : mesh.vert)
	{
		if ( vcg::tri::IsMarked(mesh, &vert) )
			local_mark.emplace(&vert);
	}

	while ( !frontier.empty() )
	{
		auto current = frontier.front(); frontier.pop();
		local_mark.emplace(current);
		subtree.emplace_back(current);

		vcg::edge::VVStarVE(current, star);
		for (auto* vert : star)
		{
			if ( local_mark.count(vert) == 0 )
				frontier.push(vert);
		}
	}

	return subtree;
}

static SkeletonVertex* getClosestNonMarkedNonTwoConnectedNeighbor(SkeletonMesh& mesh, SkeletonVertex* vertex);

std::pair<SkeletonVertex*, SkeletonVertex*> getClosestNonTwoConnectedNeighbors(SkeletonMesh& mesh, SkeletonVertex* vertex)
{
	assert(vcg::edge::VEDegree<SkeletonEdge>(vertex) == 2);
	vcg::tri::UnMarkAll(mesh);

	std::vector<SkeletonVertex*> star;
	vcg::edge::VVStarVE(vertex, star);
	vcg::tri::Mark(mesh, vertex);

	return {
		getClosestNonMarkedNonTwoConnectedNeighbor(mesh, star[0]),
		getClosestNonMarkedNonTwoConnectedNeighbor(mesh, star[1])
	};
}

static SkeletonVertex* getClosestNonMarkedNonTwoConnectedNeighbor(SkeletonMesh& mesh, SkeletonVertex* vertex)
{
	auto* current = vertex;
	std::vector<SkeletonVertex*> star;
	while ( vcg::edge::VEDegree<SkeletonEdge>(current) == 2 )
	{
		vcg::tri::Mark(mesh, current);
		vcg::edge::VVStarVE(current, star);
		if ( vcg::tri::IsMarked(mesh, star[0]) )
			current = star[1];
		else
			current = star[0];
	}

	return current;
}

}
