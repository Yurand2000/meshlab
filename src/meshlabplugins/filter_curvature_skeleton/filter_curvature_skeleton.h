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

#ifndef FILTERCURVATURESKELETON_PLUGIN
#define FILTERCURVATURESKELETON_PLUGIN

#include <memory>
#include <common/plugins/interfaces/filter_plugin.h>
#include "common/TemplateFilter.h"

namespace curvatureSkeleton
{

class FilterCurvatureSkeleton : public QObject, public FilterPlugin
{
	Q_OBJECT
	MESHLAB_PLUGIN_IID_EXPORTER(FILTER_PLUGIN_IID)
	Q_INTERFACES(FilterPlugin)

public:
	typedef std::vector<std::unique_ptr<TemplateFilter>> Filters;

public:
	// possible filters
	enum : MeshLabPlugin::ActionIDType {
		SKELETONIZE,
		PRUNE_SKELETON,
		TREE_SEGMENTATION,

		PRUNE_AND_EXTEND,
		COMPUTE_ORDER_NUMBERS,
		BRANCH_CUTTER,
	};

	FilterCurvatureSkeleton();

	QString pluginName() const override;

	QString filterName(ActionIDType filter) const override;
	QString pythonFilterName(ActionIDType f) const override;
	QString filterInfo(ActionIDType filter) const override;
	FilterClass getClass(QAction const* a) const override;
	FilterArity filterArity(QAction const* a) const override;
	int getPreConditions(QAction const* a) const override;
	int postCondition(QAction const* a) const override;
	RichParameterList initParameterList(QAction const*, MeshDocument const&) override;
	std::map<std::string, QVariant> applyFilter(
		QAction const*,
		RichParameterList const&,
		MeshDocument&,
		unsigned int&,
		vcg::CallBackPos*
	) override;

private:
	Filters filters;

private:
	template<class T>
	T findFilterAndExecute(ActionIDType filter_id, std::function<T(TemplateFilter&)> func, std::function<T()> def)
	{
		for (auto& filter : filters) {
			if (filter->isValidFilter(filter_id))
				return func(*filter);
		}
		return def();
	}

	template<class T>
	T findFilterAndExecute(ActionIDType filter_id, std::function<T(TemplateFilter const&)> func, std::function<T()> def) const
	{
		for (auto& filter : filters) {
			if (filter->isValidFilter(filter_id))
				return func(*filter);
		}
		return def();
	}

	template<class T>
	T findFilterAndExecute(QAction const* a, std::function<T(TemplateFilter&)> func, std::function<T()> def)
	{
		return findFilterAndExecute<T>(ID(a), func, def);
	}

	template<class T>
	T findFilterAndExecute(QAction const* a, std::function<T(TemplateFilter const&)> func, std::function<T()> def) const
	{
		return findFilterAndExecute<T>(ID(a), func, def);
	}
};

}

#endif
