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

#include "BranchMeasureFilter.h"

#include "common/AlgorithmSkeletonize.h"
#include "common/BranchExtender.h"
#include "common/SkeletonMesh.h"

namespace curvatureSkeleton
{

static Scalarm computeLongestPath(SkeletonMesh& mesh, SkeletonVertex* base);

std::map<std::string, QVariant> BranchMeasureFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos* cb)
{
	//get parameters
	auto only_selected = params.getBool(PARAM_MEASURE_ONLY_SELECTED);
	auto save_skeletons = params.getBool(PARAM_SAVE_SKELETONS);

	std::vector<MeshModel*> models;
	if (only_selected)
	{
		models.emplace_back(document.mm());
	}
	else
	{
		for (auto& mesh_mm : document.meshIterator())
		{
			if ( mesh_mm.isVisible() )
				models.emplace_back(&mesh_mm);
		}
	}

	//foreach mesh
	for (int i = 0; i < models.size(); i++)
	{
		auto* mesh_mm = models[i];
		auto& mesh = mesh_mm->cm;

		cb( (i + 1) * 100.0 / models.size(), QString("Measuring %1...").arg(mesh_mm->label()).toStdString().c_str());

		//compute skeleton
		auto params = AlgorithmSkeletonize::Parameters();
		params.max_iterations = 200;
		params.skeletonizer_params = AlgorithmSkeletonize::Skeletonizer::Parameters(mesh, 0.002);
		params.skeletonizer_params.medially_centering_speed_tradeoff = 50;
		params.skeletonizer_params.quality_speed_tradeoff = 40;
		params.save_mesoskeletons = false;

		auto skeleton = AlgorithmSkeletonize(vcg::DummyCallBackPos, plugin).skeletonize(mesh, params, false);

		//extend skeleton
		Scalarm cone_extension_angle = 25.f;
		int search_depth = 20;
		BranchExtender::extendLeafs(mesh, skeleton, vcg::math::ToRad(cone_extension_angle), search_depth);

		//convert skeleton to SkeletonMesh
		SkeletonMesh c_skeleton;
		vcg::tri::Append<SkeletonMesh, CMeshO>::MeshCopyConst(c_skeleton, skeleton);
		vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(c_skeleton);
		vcg::tri::InitVertexIMark(c_skeleton);

		Scalarm longest_path = 0.0;
		for (auto& vert : c_skeleton.vert)
		{
			if ( !vert.IsD() && vcg::edge::VEDegree<SkeletonEdge>(&vert) == 1 )
				longest_path = std::max(longest_path, computeLongestPath(c_skeleton, &vert));
		}

		plugin.log(
			QString("%2 - Curved Lenght: %1")
			.arg(longest_path, 0, 'f', 3)
			.arg(mesh_mm->label())
			.toStdString()
		);

		//save skeleton
		if (save_skeletons)
		{
			auto* save_skeleton_mm = document.addNewMesh(QString(), QString("%1 - Skeleton").arg(mesh_mm->label()), false);
			vcg::tri::Append<CMeshO, CMeshO>::MeshCopyConst(save_skeleton_mm->cm, skeleton);
			save_skeleton_mm->updateBoxAndNormals();
		}
	}

	return {};
}

Scalarm computeLongestPath(SkeletonMesh& mesh, SkeletonVertex* base)
{
	std::queue< SkeletonVertex* >frontier;
	auto longest_path = vcg::tri::Allocator<SkeletonMesh>::AddPerVertexAttribute<Scalarm>(mesh);
	for (auto& vert : mesh.vert)
		longest_path[vert] = 0;

	Scalarm longest_path_lenght = 0.0;

	vcg::tri::UnMarkAll(mesh);

	frontier.emplace(base);
	std::vector<SkeletonVertex*> star;
	while (!frontier.empty())
	{
		auto* current = frontier.front();
		frontier.pop();

		vcg::tri::Mark(mesh, current);
		vcg::edge::VVStarVE(current, star);
		for (auto* adj : star)
		{
			if( vcg::tri::IsMarked(mesh, adj) )
				continue;

			frontier.push(adj);

			auto distance = vcg::Distance(current->cP(), adj->cP());
			longest_path[adj] = longest_path[current] + distance;
			longest_path_lenght = std::max(longest_path_lenght, longest_path[adj]);
		}
	}

	vcg::tri::Allocator<SkeletonMesh>::DeletePerVertexAttribute(mesh, longest_path);
	return longest_path_lenght;
}

}
