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

#include "filter_template.h"

templateFilter::templateFilter(MeshLabPlugin::ActionIDType id, const char* dn, const char* dd,
	FilterPlugin::FilterClass cat, const char* pn, FilterPlugin::FilterArity ar, int pre, int post) :
		filter_id(id),
		display_name(dn),
		description(dd),
		category(cat),
		python_name(pn),
		filter_arity(ar),
		preconditions(pre),
		postconditions(post)
{ }

templateFilter::~templateFilter() { }

bool templateFilter::isValidFilter(MeshLabPlugin::ActionIDType filter) const { return filter == filter_id; }

QString templateFilter::filterName() const { return display_name; }
QString templateFilter::pythonFilterName() const { return python_name; }
QString templateFilter::filterInfo() const { return description; }

FilterPlugin::FilterClass templateFilter::getClass() const { return category; }
FilterPlugin::FilterArity templateFilter::filterArity() const { return filter_arity; }

int templateFilter::getPreConditions() const { return preconditions; }
int templateFilter::postCondition() const { return postconditions; }
