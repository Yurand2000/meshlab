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

#ifndef FILTERCURVATURESKELETON_FILTER_RESTORE_HACK_DATA
#define FILTERCURVATURESKELETON_FILTER_RESTORE_HACK_DATA

#include "common/TemplateFilter.h"

#include <common/plugins/interfaces/filter_plugin.h>

#define PARAM_ORIGINAL_MESH "original_mesh"
#define PARAM_FACE_TAG_ID "facetag_id"
#define PARAM_HOLE_ADJ_TAG_ID "hole_adj_facetag_id"

namespace curvatureSkeleton
{

class RestoreHackOrderDataFilter : public TemplateFilter
{
public:
	RestoreHackOrderDataFilter();

	virtual RichParameterList initParameterList(FilterPlugin const&, MeshDocument const&) override;
	virtual std::map<std::string, QVariant> applyFilter(
		FilterPlugin const&,
		RichParameterList const&,
		MeshDocument&,
		unsigned int&,
		vcg::CallBackPos*) override;
};

}

#endif //FILTERCURVATURESKELETON_FILTER_RESTORE_HACK_DATA
