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

#include "MeasuresComputeFilter.h"

#include <vcg/complex/algorithms/create/platonic.h>
#include <vector>
#include <stack>
#include <algorithm>
#include "common/SkeletonMesh.h"
#include "common/BranchTagger.h"
#include <meshlabplugins/filter_select/meshselect.h>
#include <common/plugins/plugin_manager.h>

#define ATTRIBUTE_ROOT_INDEX "root_index"
#define ATTRIBUTE_BRANCH_NUMBER "branch_number"
#define ATTRIBUTE_MESH_TO_SKELETON "skeleton_index"
#define ATTRIBUTE_STRAHLER_NUMBER "strahler_number"
#define ATTRIBUTE_HACK_ORDER_NUMBER "hack_order_number"

namespace curvatureSkeleton
{

struct Branch
{
	int start_index;
	int end_index;
	Scalarm order_number;
	int parent_branch_index;
};

typedef vcg::tri::Allocator<CMeshO>            CMeshOAllocator;
typedef vcg::tri::Allocator<SkeletonMesh>      SkeletonAllocator;
typedef vcg::tri::Append<SkeletonMesh, CMeshO> CMeshOToSkeletonAppend;
typedef vcg::tri::Append<CMeshO, SkeletonMesh> SkeletonToCMeshOAppend;
typedef vcg::tri::UpdateTopology<SkeletonMesh> SkeletonTopology;

std::map<std::string, QVariant> MeasuresComputeFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos* cb)
{
	auto  original_mm  = document.getMesh(params.getMeshId(PARAM_ORIGINAL_MESH));
	auto& original     = original_mm->cm;
	auto  skeleton_mm  = document.getMesh(params.getMeshId(PARAM_SKELETON_MESH));
	auto& skeleton     = skeleton_mm->cm;
	auto  tree_mm      = document.getMesh(params.getMeshId(PARAM_TREE_MESH));
	auto& tree         = tree_mm->cm;

	auto attribute_idx = params.getEnum(PARAM_ATTRIBUTE);
	auto attribute     = (attribute_idx == 0) ? ATTRIBUTE_HACK_ORDER_NUMBER : ATTRIBUTE_STRAHLER_NUMBER;
	auto ascending     = (attribute_idx == 0);

	int  skeleton_root_index = CMeshOAllocator::GetPerMeshAttribute<Scalarm>(skeleton, ATTRIBUTE_ROOT_INDEX)();
	int  tree_root_index = CMeshOAllocator::GetPerMeshAttribute<Scalarm>(tree, ATTRIBUTE_ROOT_INDEX)();

	SkeletonMesh converted_tree;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_tree, tree);
	SkeletonTopology::VertexEdge(converted_tree);
	vcg::tri::InitVertexIMark(converted_tree);
	vcg::tri::InitEdgeIMark(converted_tree);

	//make branches from attribute values.
	auto less = detail::getCompareFunction(ascending);

	auto order_numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(tree, attribute);
	auto branch_numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(tree, ATTRIBUTE_BRANCH_NUMBER);
	std::vector<Branch> branches;
	std::stack<SkeletonVertex*> frontier;

	frontier.push( &converted_tree.vert[tree_root_index] );
	branches.push_back({ tree_root_index, tree_root_index, order_numbers[tree_root_index], -1 });
	branch_numbers[tree_root_index] = 0;

	vcg::tri::UnMarkAll(converted_tree);
	do
	{
		auto* node   = frontier.top();
		int   branch = branch_numbers[node->Index()];
		vcg::tri::Mark(converted_tree, node);
		frontier.pop();

		auto order_number = order_numbers[node->Index()];

		std::vector<SkeletonVertex*> verts;
		vcg::edge::VVStarVE(node, verts);
		for (auto* adj : verts)
		{
			if (!vcg::tri::IsMarked(converted_tree, adj))
			{
				auto adj_index        = adj->Index();
				auto adj_order_number = order_numbers[adj_index];

				if ( less(adj_order_number, order_number) )
				{
					branches.push_back({ node->Index(), adj_index, adj_order_number, branch });
					branch_numbers[adj_index] = branches.size() - 1;
				}
				else
				{
					branches[branch].end_index = adj_index;
					branch_numbers[adj_index] = branch;
				}

				frontier.push(adj);
			}
		}
	}
	while ( !frontier.empty() );

	//propagate from tree to original mesh	
	BranchTagger<CMeshO>::copyAttributeTreeToSkeleton(skeleton, tree, tree_root_index, ATTRIBUTE_BRANCH_NUMBER, true);
	BranchTagger<CMeshO>::copyAttributeUsingAdjacency(skeleton, original, ATTRIBUTE_BRANCH_NUMBER, ATTRIBUTE_MESH_TO_SKELETON);

	//print number of branches
	plugin.log( std::string("** Number of branches: ") + std::to_string(branches.size()) );

	//compute lenghts
	auto original_numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(original, attribute);
	auto skeleton_numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(skeleton, attribute);

	//debug sphere
	CMeshO sphere;
	vcg::tri::Sphere<CMeshO>(sphere, 3);

	//make debug sphere mesh
	auto debug_spheres = document.addNewMesh("", "DebugSpheres", false);
	debug_spheres->cm.face.EnableFFAdjacency();
	debug_spheres->updateDataMask(MeshModel::MM_FACEFACETOPO);

	plugin.log("** Branch lengths: ");
	plugin.log("*  Smp = Simple lenght; Sph = Sphere radius; Len = Skeleton Lenght  *");
	Scalarm simple_tle = 0;
	Scalarm tle = 0;
	for (int i = 0; i < branches.size(); i++)
	{
		auto& branch = branches[i];
		auto  tip_number = order_numbers[branch.end_index];

		Scalarm length = 0;
		auto* skeleton_base = &skeleton.vert[detail::getVertexIndexInMesh(tree.vert[branch.start_index].P(), skeleton)];
		auto* skeleton_tip = &skeleton.vert[detail::getVertexIndexInMesh(tree.vert[branch.end_index].P(), skeleton)];
		auto path = detail::findPath(skeleton, skeleton_base, skeleton_tip);

		auto current_end = 0;
		while ( current_end < (path.size() - 1) && skeleton_numbers[path[current_end]] == tip_number )
		{
			length += vcg::Distance(  path[ current_end ]->P(), path[ current_end + 1 ]->P()  );
			current_end++;
		}

		Scalarm simple_length = vcg::Distance( path[0]->P(), path[current_end]->P() );

		auto h = vcg::Histogram<Scalarm>();
		h.Clear();
		h.SetRange(0, original.bbox.Diag(), 10000);

		auto number     = order_numbers[branch.start_index];
		auto sph_center = path[current_end]->P();
		for (auto& vertex : original.vert)
		{
			auto vnumber = original_numbers[vertex];
			if (vnumber == number)
			{
				auto distance = vcg::Distance(vertex.P(), sph_center);
				h.Add(distance);
			}
		}

		Scalarm sph_radius = 0;
		if (i != 0)
		{
			sph_radius = h.Percentile(0.005);

			// create debug spheres
			CMeshO curr_sphere;
			vcg::tri::Append<CMeshO, CMeshO>::MeshCopyConst(curr_sphere, sphere);
			vcg::tri::UpdatePosition<CMeshO>::Scale(curr_sphere, sph_radius);
			vcg::tri::UpdatePosition<CMeshO>::Translate(curr_sphere, sph_center);
			vcg::tri::Append<CMeshO, CMeshO>::MeshAppendConst(debug_spheres->cm, curr_sphere);
		}

		simple_tle += simple_length - sph_radius;
		tle += length - sph_radius;

		plugin.log("  Branch #" + std::to_string(i) + ": " + std::to_string(simple_length - sph_radius) + ";  [Smp:" + std::to_string(simple_length) + "; Sph: " + std::to_string(sph_radius) + "; Len: " + std::to_string(length) + "]");
	}
	plugin.log("  Simple TLE: " + std::to_string(simple_tle));
	plugin.log("  TLE: " + std::to_string(tle));

	//compute angles
	plugin.log("** Branch angles: ");

	auto& branch         = branches[0];
	auto  direction      = tree.vert[branch.end_index].cP() - tree.vert[branch.start_index].cP();
	auto  geotropy_angle = vcg::Angle(direction, vcg::Point3<Scalarm>(0, 1, 0));
	plugin.log( "  Geotropic Branch #0: " + std::to_string( vcg::math::ToDeg(geotropy_angle) ) );
	for (int i = 1; i < branches.size(); i++)
	{
		auto& branch = branches[i];
		auto& parent_branch = branches[branch.parent_branch_index];

		auto direction = tree.vert[branch.end_index].cP() - tree.vert[branch.start_index].cP();
		auto parent_direction = tree.vert[parent_branch.end_index].cP() - tree.vert[parent_branch.start_index].cP();
		auto angle = vcg::Angle(direction, parent_direction);
		auto geotropy_angle = vcg::Angle(direction, vcg::Point3<Scalarm>(0, 1, 0));

		plugin.log( "  Branch #" + std::to_string(i) + " with Parent #" + std::to_string(branch.parent_branch_index) + ": " + std::to_string( vcg::math::ToDeg(angle) ) );
		plugin.log( "  Geotropic Branch #" + std::to_string(i) + ": " + std::to_string( vcg::math::ToDeg(geotropy_angle) ) );
	}

	//compute tickness

	//find select plugin
	FilterPlugin* select_plugin = nullptr;
	auto it_range = meshlab::pluginManagerInstance().filterPluginIterator();
	for (auto it = it_range.begin(); it != it_range.end(); it++)
	{
		if ((*it)->pluginName() == "FilterSelect")
		{
			select_plugin = *it;
			break;
		}
	}

	if ( select_plugin == nullptr )
		return {};

	//save individual branches
	auto* current_mesh = document.mm();
	for (int i = 0; i < branches.size(); i++)
	{
		auto* branch_mm = document.addNewMesh("", "Branch #" + QString::number(i), true);
		auto& branch    = branch_mm->cm;

		auto branch_numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(branch, ATTRIBUTE_BRANCH_NUMBER);
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopyConst(branch, original);
		branch.vert.EnableVFAdjacency();
		branch.face.EnableVFAdjacency();
		vcg::tri::UpdateTopology<CMeshO>::VertexFace(branch);

		vcg::tri::UpdateSelection<CMeshO>::VertexClear(branch);
		for (auto& vertex : branch.vert)
		{
			if (!vertex.IsD() && branch_numbers[vertex] == i)
			{
				vertex.SetS();

				std::vector<CMeshO::VertexPointer> adjs;
				vcg::face::VVStarVF<CMeshO::FaceType>(&vertex, adjs);

				for (auto* adj : adjs)
				{
					if (!adj->IsD())
						adj->SetS();
				}
			}

		}
		branch.svn = vcg::tri::UpdateSelection<CMeshO>::VertexInvert(branch);
		branch.vert.DisableVFAdjacency();
		branch.face.DisableVFAdjacency();

		//delete vertices via filter
		uint post_condition_mask;
		select_plugin->applyFilter(
			select_plugin->getFilterAction(SelectionFilterPlugin::FP_SELECT_DELETE_VERT),
			RichParameterList(),
			document,
			post_condition_mask,
			cb
		);
	}
	document.setCurrent(current_mesh);

	// colorize by branch (temp)
	auto colors = BranchTagger<CMeshO>::generateColorsFromAttribute(tree, ATTRIBUTE_BRANCH_NUMBER);
	std::random_shuffle(colors.begin(), colors.end());

	tree_mm->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
	BranchTagger<CMeshO>::colorizeByAttribute(tree, colors, ATTRIBUTE_BRANCH_NUMBER);

	skeleton_mm->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
	BranchTagger<CMeshO>::colorizeByAttribute(skeleton, colors, ATTRIBUTE_BRANCH_NUMBER);

	original_mm->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
	BranchTagger<CMeshO>::colorizeByAttribute(original, colors, ATTRIBUTE_BRANCH_NUMBER);
	
	return {};
}

}
