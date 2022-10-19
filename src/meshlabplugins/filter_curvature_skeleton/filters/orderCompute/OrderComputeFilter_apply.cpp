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

#include "OrderComputeFilter.h"

#include "common/SkeletonMesh.h"
#include "common/BranchTagger.h"
#include "common/Utils.h"
#include <vcg/complex/algorithms/create/platonic.h>
#include <vcg/complex/algorithms/update/color.h>
#include <vcg/complex/algorithms/stat.h>
#include <meshlabplugins/filter_select/meshselect.h>
#include <common/plugins/plugin_manager.h>

#define ATTRIBUTE_STRAHLER_NUMBER "strahler_number"
#define ATTRIBUTE_HACK_ORDER_NUMBER "hack_order_number"
#define ATTRIBUTE_ROOT_INDEX "root_index"
#define ATTRIBUTE_MESH_TO_SKELETON "skeleton_index"
#define ATTRIBUTE_BRANCH_NUMBER "branch_number"

namespace curvatureSkeleton
{

typedef vcg::tri::Allocator<CMeshO>            CMeshOAllocator;
typedef vcg::tri::Allocator<SkeletonMesh>      SkeletonAllocator;
typedef vcg::tri::Append<SkeletonMesh, CMeshO> CMeshOToSkeletonAppend;
typedef vcg::tri::Append<CMeshO, SkeletonMesh> SkeletonToCMeshOAppend;
typedef vcg::tri::UpdateTopology<SkeletonMesh> SkeletonTopology;
typedef vcg::tri::UpdateColor<CMeshO>          UpdateColor;
typedef vcg::Color4b                           Color;

namespace detail
{
	struct OrderComputeFilterParameters
	{
	public:
		FilterPlugin const& plugin; MeshDocument& document;
		MeshModel *original_mm, *skeleton_mm, *tree_mm;
		std::string attribute; bool ascending;
		bool save_to_color;

		int skeleton_root_index, tree_root_index;

	public:
		OrderComputeFilterParameters(FilterPlugin const& plugin, MeshDocument& document, RichParameterList const& params);

		inline CMeshO& original() { return original_mm->cm; }
		inline CMeshO& skeleton() { return skeleton_mm->cm; }
		inline CMeshO& tree() { return tree_mm->cm; }
	};

	struct Branch
	{
		int start_index;
		int end_index;
		Scalarm order_number;
		int parent_branch_index;
	};

	static std::vector<Branch> computeBranches(detail::OrderComputeFilterParameters& params, SkeletonMesh& converted_tree);
	static void computeLenghts(detail::OrderComputeFilterParameters& params, std::vector<detail::Branch> const& branches);
	static void computeAngles(detail::OrderComputeFilterParameters& params, std::vector<detail::Branch> const& branches);
	static void saveIndividualBranches(detail::OrderComputeFilterParameters& params, std::vector<detail::Branch> const& branches, vcg::CallBackPos* cb);

	static FilterPlugin* findSelectPlugin();
}

std::map<std::string, QVariant> OrderComputeFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& rich_params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos*		 cb)
{
	auto params = detail::OrderComputeFilterParameters(plugin, document, rich_params);

	//meshes
	auto& original = params.original();
	auto& skeleton = params.skeleton();
	auto& tree = params.tree();

	//convert the tree to SkeletonMesh
	SkeletonMesh converted_tree;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_tree, tree);
	SkeletonTopology::VertexEdge(converted_tree);
	vcg::tri::InitVertexIMark(converted_tree);
	vcg::tri::InitEdgeIMark(converted_tree);

	//compute order number
	if(params.attribute == ATTRIBUTE_HACK_ORDER_NUMBER)
		vcg::tri::Stat<SkeletonMesh>::ComputeHackOrderNumbers(converted_tree, params.tree_root_index, ATTRIBUTE_HACK_ORDER_NUMBER);
	else
		vcg::tri::Stat<SkeletonMesh>::ComputeStrahlerNumbers(converted_tree, params.tree_root_index, ATTRIBUTE_STRAHLER_NUMBER);

	//update tree mesh
	{
		CMeshOAllocator::AddPerVertexAttribute<Scalarm>(tree, params.attribute);
		CMeshOAllocator::GetPerMeshAttribute<Scalarm>(tree, ATTRIBUTE_ROOT_INDEX)() = params.tree_root_index;
		SkeletonToCMeshOAppend::MeshCopyConst(tree, converted_tree);
	}

	//save attributes back to the original meshes
	BranchTagger<CMeshO>::copyAttributeTreeToSkeleton(skeleton, tree, params.tree_root_index, params.attribute, params.ascending);
	BranchTagger<CMeshO>::copyAttributeUsingAdjacency(skeleton, original, params.attribute, ATTRIBUTE_MESH_TO_SKELETON);

	//colorize meshes
	if (params.save_to_color)
	{
		auto colors = BranchTagger<CMeshO>::generateColorsFromAttribute(tree, params.attribute);

		params.tree_mm->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
		BranchTagger<CMeshO>::colorizeByAttribute(tree, colors, params.attribute);

		params.skeleton_mm->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
		BranchTagger<CMeshO>::colorizeByAttribute(skeleton, colors, params.attribute);

		params.original_mm->updateDataMask(MeshModel::MeshElement::MM_VERTCOLOR);
		BranchTagger<CMeshO>::colorizeByAttribute(original, colors, params.attribute);
	}

	auto branches = detail::computeBranches(params, converted_tree);

	//propagate from tree to original mesh	
	BranchTagger<CMeshO>::copyAttributeTreeToSkeleton(skeleton, tree, params.tree_root_index, ATTRIBUTE_BRANCH_NUMBER, true);
	BranchTagger<CMeshO>::copyAttributeUsingAdjacency(skeleton, original, ATTRIBUTE_BRANCH_NUMBER, ATTRIBUTE_MESH_TO_SKELETON);

	detail::computeLenghts(params, branches);
	detail::computeAngles(params, branches);
	//compute tickness

	detail::saveIndividualBranches(params, branches, cb);

	return {};
}

detail::OrderComputeFilterParameters::OrderComputeFilterParameters(FilterPlugin const& plugin, MeshDocument& document, RichParameterList const& params)
	: plugin(plugin), document(document)
{
	original_mm = document.getMesh(params.getMeshId(PARAM_ORIGINAL_MESH));
	skeleton_mm = document.getMesh(params.getMeshId(PARAM_SKELETON_MESH));
	tree_mm = document.getMesh(params.getMeshId(PARAM_TREE_MESH));

	auto attribute_idx = params.getEnum(PARAM_ATTRIBUTE);
	attribute = (attribute_idx == 0) ? ATTRIBUTE_HACK_ORDER_NUMBER : ATTRIBUTE_STRAHLER_NUMBER;
	ascending = (attribute_idx == 0);
	save_to_color = params.getBool(PARAM_ATTRIBUTE_TO_COLOR);

	skeleton_root_index = CMeshOAllocator::GetPerMeshAttribute<Scalarm>(skeleton(), ATTRIBUTE_ROOT_INDEX)();
	tree_root_index     = CMeshOAllocator::GetPerMeshAttribute<Scalarm>(tree(), ATTRIBUTE_ROOT_INDEX)();
}

std::vector<detail::Branch> detail::computeBranches(detail::OrderComputeFilterParameters& params, SkeletonMesh& converted_tree)
{
	//param references
	auto& tree = params.tree();
	auto& plugin = params.plugin;
	auto attribute = params.attribute;
	auto ascending = params.ascending;
	auto tree_root_index = params.tree_root_index;

	auto less = detail::getCompareFunction(ascending);

	auto order_numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(tree, attribute);
	auto branch_numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(tree, ATTRIBUTE_BRANCH_NUMBER);
	std::vector<detail::Branch> branches;
	std::stack<SkeletonVertex*> frontier;

	frontier.push(&converted_tree.vert[tree_root_index]);
	branches.push_back({ tree_root_index, tree_root_index, order_numbers[tree_root_index], -1 });
	branch_numbers[tree_root_index] = 0;

	vcg::tri::UnMarkAll(converted_tree);
	do
	{
		auto* node = frontier.top();
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
				auto adj_index = adj->Index();
				auto adj_order_number = order_numbers[adj_index];

				if (less(adj_order_number, order_number))
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
	} while (!frontier.empty());
	
	plugin.log(std::string("** Number of branches: ") + std::to_string(branches.size()));
	return branches;
}

void detail::computeLenghts(detail::OrderComputeFilterParameters& params, std::vector<detail::Branch> const& branches)
{
	//meshes
	auto& original = params.original();
	auto& skeleton = params.skeleton();
	auto& tree = params.tree();
	auto& plugin = params.plugin;
	auto& document = params.document;

	//make debug sphere mesh
	CMeshO sphere;
	vcg::tri::Sphere<CMeshO>(sphere, 3);

	auto debug_spheres = document.addNewMesh("", "DebugSpheres", false);
	debug_spheres->cm.face.EnableFFAdjacency();
	debug_spheres->updateDataMask(MeshModel::MM_FACEFACETOPO);

	//compute lenghts
	auto order_numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(tree, params.attribute);
	auto original_numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(original, params.attribute);
	auto skeleton_numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(skeleton, params.attribute);
	plugin.log("** Branch lengths: ");
	plugin.log("*  Smp = Simple lenght; Sph = Sphere radius; Len = Skeleton Lenght  *");
	Scalarm simple_tle = 0;
	Scalarm tle = 0;
	for (int i = 0; i < branches.size(); i++)
	{
		auto& branch = branches[i];
		auto  tip_number = order_numbers[branch.end_index];

		Scalarm length = 0;
		auto* skeleton_base = &skeleton.vert[
			Utils<CMeshO>::getVertexIndexInMesh(
				tree.vert[branch.start_index].P(), skeleton
		)];
		auto* skeleton_tip = &skeleton.vert[
			Utils<CMeshO>::getVertexIndexInMesh(
				tree.vert[branch.end_index].P(), skeleton
		)];

		auto path = detail::findPath(skeleton, skeleton_base, skeleton_tip);

		auto current_end = 0;
		while (current_end < (path.size() - 1) && skeleton_numbers[path[current_end]] == tip_number)
		{
			length += vcg::Distance(path[current_end]->P(), path[current_end + 1]->P());
			current_end++;
		}

		Scalarm simple_length = vcg::Distance(path[0]->P(), path[current_end]->P());

		auto h = vcg::Histogram<Scalarm>();
		h.Clear();
		h.SetRange(0, original.bbox.Diag(), 10000);

		auto number = order_numbers[branch.start_index];
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
}

void detail::computeAngles(detail::OrderComputeFilterParameters& params, std::vector<detail::Branch> const& branches)
{
	//references
	auto& tree = params.tree();
	auto& plugin = params.plugin;

	plugin.log("** Branch angles: ");

	auto& branch = branches[0];
	auto  direction = tree.vert[branch.end_index].cP() - tree.vert[branch.start_index].cP();
	auto  geotropy_angle = vcg::Angle(direction, vcg::Point3<Scalarm>(0, 1, 0));
	plugin.log("  Geotropic Branch #0: " + std::to_string(vcg::math::ToDeg(geotropy_angle)));
	for (int i = 1; i < branches.size(); i++)
	{
		auto& branch = branches[i];
		auto& parent_branch = branches[branch.parent_branch_index];

		auto direction = tree.vert[branch.end_index].cP() - tree.vert[branch.start_index].cP();
		auto parent_direction = tree.vert[parent_branch.end_index].cP() - tree.vert[parent_branch.start_index].cP();
		auto angle = vcg::Angle(direction, parent_direction);
		auto geotropy_angle = vcg::Angle(direction, vcg::Point3<Scalarm>(0, 1, 0));

		plugin.log("  Branch #" + std::to_string(i) + " with Parent #" + std::to_string(branch.parent_branch_index) + ": " + std::to_string(vcg::math::ToDeg(angle)));
		plugin.log("  Geotropic Branch #" + std::to_string(i) + ": " + std::to_string(vcg::math::ToDeg(geotropy_angle)));
	}
}

FilterPlugin* detail::findSelectPlugin()
{
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

	return select_plugin;
}

void detail::saveIndividualBranches(detail::OrderComputeFilterParameters& params, std::vector<detail::Branch> const& branches, vcg::CallBackPos* cb)
{
	//references
	auto& document = params.document;
	auto& original = params.original();
	auto& tree = params.tree();
	auto& plugin = params.plugin;

	auto select_plugin = detail::findSelectPlugin();
	if (select_plugin == nullptr)
		return;

	auto* current_mesh = document.mm();
	original.vert.EnableVFAdjacency();
	original.face.EnableVFAdjacency();
	vcg::tri::UpdateTopology<CMeshO>::VertexFace(original);
	auto branch_numbers = CMeshOAllocator::GetPerVertexAttribute<Scalarm>(original, ATTRIBUTE_BRANCH_NUMBER);
	for (int i = 0; i < branches.size(); i++)
	{
		vcg::tri::UpdateSelection<CMeshO>::VertexClear(original);
		for (auto& vertex : original.vert)
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
		vcg::tri::UpdateSelection<CMeshO>::FaceFromVertexLoose(original);

		auto* branch_mm = document.addNewMesh("", "Branch #" + QString::number(i), true);
		auto& branch = branch_mm->cm;
		CMeshOAllocator::AddPerVertexAttribute<Scalarm>(branch, ATTRIBUTE_BRANCH_NUMBER);
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(branch, original, true);
		vcg::tri::UpdateSelection<CMeshO>::Clear(branch);
	}
	vcg::tri::UpdateSelection<CMeshO>::Clear(original);
	original.vert.DisableVFAdjacency();
	original.face.DisableVFAdjacency();
	document.setCurrent(current_mesh);
}

}
