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

#include <unordered_map>

namespace CGalAdapter
{
	struct MeshSkeletonizerParameters;

	class MeshSkeletonizer
	{
	public:
		typedef std::unordered_map<uint, uint> MeshToSkeletonVertices;

	public:
		MeshSkeletonizer(CMeshO const& input);
		MeshSkeletonizer(CMeshO const& input, MeshSkeletonizerParameters const& params);
		~MeshSkeletonizer();

		MeshSkeletonizer(MeshSkeletonizer& copy) = delete;
		MeshSkeletonizer& operator=(MeshSkeletonizer& copy) = delete;

		void   computeStep();
		bool   hasConverged();
		CMeshO getMesoSkeleton();
		CMeshO getSkeleton();
		MeshToSkeletonVertices getSkeletonVertexAssociations();

	private:
		CGALSkeletonizer* skeletonizer;
		double            delta_area_threshold;

		double            original_area;
		double            last_area;

		CGALSkeleton	  skeleton;

		void setSkeletonizerParameters(MeshSkeletonizerParameters const& params);
		void generateSkeleton();
	};

	struct MeshSkeletonizerParameters
	{
		double max_triangle_angle;
		double min_edge_length;
		double quality_speed_tradeoff;
		double medially_centering_speed_tradeoff;
		double delta_area_threshold;

		MeshSkeletonizerParameters();
		MeshSkeletonizerParameters(CMeshO const&);
	};
}

#endif
