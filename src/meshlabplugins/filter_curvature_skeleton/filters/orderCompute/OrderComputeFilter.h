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

#ifndef FILTERCURVATURESKELETON_FILTER_ORDER_COMPUTE
#define FILTERCURVATURESKELETON_FILTER_ORDER_COMPUTE

#include "common/TemplateFilter.h"

#include <common/plugins/interfaces/filter_plugin.h>

#define PARAM_ORIGINAL_MESH "original_mesh"
#define PARAM_SKELETON_MESH "skeleton_mesh"
#define PARAM_ROOT_INDEX "root_index"
#define PARAM_MIN_EDGE_SIZE "min_edge_size"
#define PARAM_MIN_EDGE_PERCENTILE "min_edge_percentile"
#define PARAM_ATTRIBUTE_TO_COLOR "attribute_to_color"

namespace curvatureSkeleton
{

class OrderComputeFilter : public TemplateFilter
{
public:
	OrderComputeFilter();

	virtual RichParameterList initParameterList(FilterPlugin const&, MeshDocument const&) override;
	virtual std::map<std::string, QVariant> applyFilter(
		FilterPlugin const&,
		RichParameterList const&,
		MeshDocument&,
		unsigned int&,
		vcg::CallBackPos*) override;
};

}

#endif //FILTERCURVATURESKELETON_FILTER_ORDER_COMPUTE
