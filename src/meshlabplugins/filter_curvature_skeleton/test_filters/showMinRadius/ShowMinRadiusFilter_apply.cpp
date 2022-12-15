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

#include "ShowMinRadiusFilter.h"

namespace curvatureSkeleton
{

std::map<std::string, QVariant> ShowMinRadiusTestFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& rich_params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos*		 cb)
{
	auto& original = document.getMesh(rich_params.getMeshId(PARAM_ORIGINAL_MESH))->cm;
	auto* skeleton_mesh = document.getMesh(rich_params.getMeshId(PARAM_SKELETON_MESH));
	auto display_data = rich_params.getEnum(PARAM_DISPLAY_TYPE);
	skeleton_mesh->updateDataMask(MeshModel::MM_VERTQUALITY);
	auto& skeleton = skeleton_mesh->cm;

	Scalarm average = 0;
	int count = 0;
	for (auto& skel_vertex : skeleton.vert)
	{
		auto min_distance = std::numeric_limits<Scalarm>::max();
		for (auto& vertex : original.vert)
		{
			auto distance = vcg::SquaredDistance(vertex.cP(), skel_vertex.cP());
			if (distance < min_distance) {
				min_distance = distance;
				skel_vertex.Q() = std::sqrt(distance);
			}
		}

		average += skel_vertex.Q();
		count += 1;
	}

	if (display_data == 1 && count > 0) {
		average /= count;

		for (auto& skel_vertex : skeleton.vert) {
			skel_vertex.Q() = vcg::math::Abs( average - skel_vertex.Q() );
		}
	}

	return {};
}

}
