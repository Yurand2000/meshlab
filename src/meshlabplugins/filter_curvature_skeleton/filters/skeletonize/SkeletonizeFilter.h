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

#ifndef FILTERCURVATURESKELETON_FILTER_SKELETONIZE
#define FILTERCURVATURESKELETON_FILTER_SKELETONIZE

#include "common/TemplateFilter.h"

#include <common/plugins/interfaces/filter_plugin.h>

// parameter names
#define PARAM_MAX_ITERATIONS "max_iterations"
#define PARAM_DELTA_AREA_TERMINATION "delta_area_convergence"
#define PARAM_GENERATE_INTERMEDIATE_MESHES "generate_intermediate_meshes"
#define PARAM_SAVE_SKELETAL_DISTANCE_TO_MESH_QUALITY "save_skeletal_distance_in_quality"
#define PARAM_MAX_ANGLE "max_triangle_angle"
#define PARAM_MIN_EDGE_LENGTH "min_edge_length"
#define PARAM_QUALITY_TRADEOFF "quality_tradeoff"
#define PARAM_ENABLE_MEDIALLY_CENTERING "enable_medially_centering"
#define PARAM_MEDIALLY_CENTERING_TRADEOFF "medially_centering_tradeoff"
#define PARAM_DO_EXTEND_BRANCHES "extend_branches"
#define PARAM_EXTENSION_CONE_ANGLE "cone_angle"

namespace curvatureSkeleton
{

class SkeletonizeFilter : public TemplateFilter
{
public:
	SkeletonizeFilter();

	virtual RichParameterList initParameterList(FilterPlugin const&, MeshModel const&) override;
	virtual std::map<std::string, QVariant> applyFilter(
		FilterPlugin const&,
		RichParameterList const&,
		MeshDocument&,
		unsigned int&,
		vcg::CallBackPos*) override;

private:
	void checkParameters(RichParameterList const&, vcg::CallBackPos&);
};

}

#endif //FILTERCURVATURESKELETON_FILTER_SKELETONIZE
