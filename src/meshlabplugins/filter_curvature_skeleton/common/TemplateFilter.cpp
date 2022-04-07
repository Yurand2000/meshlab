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

#include "TemplateFilter.h"

TemplateFilter::TemplateFilter(MeshLabPlugin::ActionIDType id, const char* dn, const char* dd,
	FilterPlugin::FilterClass cat, const char* pn, FilterPlugin::FilterArity ar, int pre, int post)
		: filter_id(id), display_name(dn), description(dd), category(cat),
		  python_name(pn), filter_arity(ar), preconditions(pre), postconditions(post)
{ }

bool TemplateFilter::isValidFilter(MeshLabPlugin::ActionIDType filter) const { return filter == filter_id; }

QString TemplateFilter::filterName() const { return display_name; }
QString TemplateFilter::pythonFilterName() const { return python_name; }
QString TemplateFilter::filterInfo() const { return description; }

FilterPlugin::FilterClass TemplateFilter::getClass() const { return category; }
FilterPlugin::FilterArity TemplateFilter::filterArity() const { return filter_arity; }

int TemplateFilter::getPreConditions() const { return preconditions; }
int TemplateFilter::postCondition() const { return postconditions; }

RichParameterList TemplateFilter::initParameterList(FilterPlugin const&, MeshModel const&)
{
	return RichParameterList();
}

RichParameterList TemplateFilter::initParameterList(FilterPlugin const& plugin, MeshDocument const& doc)
{
	return initParameterList(plugin, *(doc.mm()));
}

std::map<std::string, QVariant> TemplateFilter::applyFilter(
	FilterPlugin const&, RichParameterList const&, MeshDocument&, unsigned int&, vcg::CallBackPos*)
{
	return std::map<std::string, QVariant>();
}
