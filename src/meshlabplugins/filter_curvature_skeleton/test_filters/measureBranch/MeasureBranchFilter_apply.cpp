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

#include "MeasureBranchFilter.h"

#include "common/PolylineMesh.h"
#include "common/AlgorithmSkeletonize.h"
#include "common/BranchExtender.h"
#include "common/SkeletonMesh.h"

#define ATTRIBUTE_BRANCH_BASE_POLYLINE "base_polyline"

namespace curvatureSkeleton
{

std::map<std::string, QVariant> MeasureBranchTestFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& rich_params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos*		 cb)
{
	auto& branch = document.mm()->cm;

	//find polyline barycenter
	auto polyline = vcg::tri::Allocator<CMeshO>::GetPerVertexAttribute<Scalarm>(branch, ATTRIBUTE_BRANCH_BASE_POLYLINE);
	int count = 0; vcg::Point3<Scalarm> polyline_barycenter = vcg::Point3<Scalarm>();
	std::vector<vcg::Point3<Scalarm>> polyline_vertices;
	for (auto& vert : branch.vert)
	{
		if (polyline[vert] >= 1.0f) {
			count++;
			polyline_barycenter += vert.cP();
			polyline_vertices.emplace_back( vert.cP() );
		}
	}
	polyline_barycenter /= count;

	//compute skeleton
	auto params = AlgorithmSkeletonize::Parameters();
	params.max_iterations = 200;
	params.skeletonizer_params = AlgorithmSkeletonize::Skeletonizer::Parameters(branch, 0.002);
	params.save_mesoskeletons = false;

	auto skeleton_mesh = AlgorithmSkeletonize(*cb, plugin).skeletonize(branch, params);

	//extend skeleton
	BranchExtender::extendLeafs(branch, skeleton_mesh, vcg::math::ToRad(10.f));

	//convert skeleton to SkeletonMesh
	SkeletonMesh skeleton;
	vcg::tri::Append<SkeletonMesh, CMeshO>::MeshCopyConst(skeleton, skeleton_mesh);

	//find base vertex on skeleton
	auto base_vert = 0;
	auto base_distance = std::numeric_limits<Scalarm>::max();
	for (int i = 0; i < skeleton.vert.size(); i++)
	{
		auto& vert = skeleton.vert[i];

		auto distance = vcg::SquaredDistance(polyline_barycenter, vert.cP());
		if (distance < base_distance) {
			base_distance = distance;
			base_vert = i;
		}
	}

	//find tip vertex on skeleton (computes also the curved lenght)
	vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(skeleton);

	auto top_vert = 0;
	auto top_distance = std::numeric_limits<Scalarm>::min();

	vcg::tri::UnMarkAll(skeleton);
	std::queue<std::pair<int, Scalarm>> frontier;
	frontier.emplace(base_vert, 0.0);

	//breadth first search
	while (!frontier.empty())
	{
		auto top = frontier.front(); frontier.pop();
		auto& vert = skeleton.vert[top.first]; auto base_distance = top.second;
		vcg::tri::Mark(skeleton, &vert);

		if (base_distance > top_distance) {
			top_distance = base_distance;
			top_vert = vert.Index();
		}

		std::vector<SkeletonMesh::VertexPointer> star;
		vcg::edge::VVStarVE(&vert, star);

		for (auto& adj : star)
		{
			if (!vcg::tri::IsMarked(skeleton, adj)) {
				auto adj_distance = vcg::Distance(vert.cP(), adj->cP()) + base_distance;

				frontier.emplace(adj->Index(), adj_distance);
			}
		}
	}

	plugin.log( QString::asprintf("Curved Lenght: %.3f", (float)top_distance).toStdString() );

	//compute linear lenghts average
	Scalarm linear_distance = 0;
	for (auto& point : polyline_vertices)
		linear_distance += vcg::Distance(point, skeleton.vert[top_vert].cP());
	linear_distance = linear_distance / polyline_vertices.size();

	plugin.log(QString::asprintf("Linear Surface Lenght: %.3f", (float)linear_distance).toStdString());

	return {};
}

}
