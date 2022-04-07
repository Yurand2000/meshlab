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

#ifndef FILTERCURVATURESKELETON_FILTER_TEMPLATE
#define FILTERCURVATURESKELETON_FILTER_TEMPLATE

#include "common/plugins/interfaces/filter_plugin.h"

class TemplateFilter
{
public:
	TemplateFilter(
		MeshLabPlugin::ActionIDType filter_id,
		const char*                 display_name,
		const char*                 description,
		FilterPlugin::FilterClass   category,
		const char*                 python_name,
		FilterPlugin::FilterArity   filter_arity,
		int                         preconditions,
		int                         postconditions);
	virtual ~TemplateFilter() = default;

	bool isValidFilter(MeshLabPlugin::ActionIDType filter) const;

	QString filterName() const;
	QString pythonFilterName() const;
	QString filterInfo() const;

	FilterPlugin::FilterClass getClass() const;
	FilterPlugin::FilterArity filterArity() const;

	int getPreConditions() const;
	int postCondition() const;

	//override just one of the two functions, as needed
	virtual RichParameterList initParameterList(FilterPlugin const&, MeshModel const&);
	virtual RichParameterList initParameterList(FilterPlugin const&, MeshDocument const&);

	virtual std::map<std::string, QVariant> applyFilter(
		FilterPlugin const&,
		RichParameterList const&,
		MeshDocument&,
		unsigned int&,
		vcg::CallBackPos*);

private:
	MeshLabPlugin::ActionIDType filter_id;
	QString                     display_name;
	QString                     description;
	FilterPlugin::FilterClass   category;
	QString                     python_name;
	FilterPlugin::FilterArity   filter_arity;
	int                         preconditions;
	int                         postconditions;
};

#endif
