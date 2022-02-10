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

#ifndef FILTERCURVATURESKELETON_CGAL_MESH_SKELETONIZER
#define FILTERCURVATURESKELETON_CGAL_MESH_SKELETONIZER

#include "typedefs.h"
#include "../common/ml_document/mesh_document.h"

namespace CGalAdapter
{
	struct MeshSkeletonizerParameters;

	class MeshSkeletonizer
	{
	public:
		MeshSkeletonizer(CMeshO const& input);
		MeshSkeletonizer(CMeshO const& input, MeshSkeletonizerParameters const& params);
		~MeshSkeletonizer();

		void   computeStep();
		bool   hasConverged();
		CMeshO getMesoSkeleton();
		CMeshO getSkeleton();

	private:
		CGALSkeletonizer* skeletonizer;
	};

	struct MeshSkeletonizerParameters
	{
		double max_triangle_angle;
		double min_edge_length;
		double quality_speed_tradeoff;
		double medially_centered_speed_tradeoff;

		MeshSkeletonizerParameters();
		MeshSkeletonizerParameters(
			double max_triangle_angle,
			double min_edge_length,
			double quality_speed_tradeoff,
			double medially_centered_speed_tradeoff
		);
	};
}

#endif
