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

#include <common/plugins/interfaces/filter_plugin.h>

class FilterCurvatureSkeleton : public QObject, public FilterPlugin
{
	Q_OBJECT
	MESHLAB_PLUGIN_IID_EXPORTER(FILTER_PLUGIN_IID)
	Q_INTERFACES(FilterPlugin)

public:
	//possible filters
	enum {
		CURVATURE_SKELETON
	};

	FilterCurvatureSkeleton();
	virtual ~FilterCurvatureSkeleton();

	QString pluginName() const override;

	QString filterName(ActionIDType filter) const override;
	QString pythonFilterName(ActionIDType f) const override;
	QString filterInfo(ActionIDType filter) const override;
	FilterClass getClass(QAction const* a) const override;
	FilterArity filterArity(QAction const* a) const override;
	int getPreConditions(QAction const* a) const override;
	int postCondition(QAction const* a) const override;
	RichParameterList initParameterList(QAction const*, MeshModel const&) override;
	std::map<std::string, QVariant> applyFilter(
		QAction const*,
		RichParameterList const&,
		MeshDocument&,
		unsigned int&,
		vcg::CallBackPos*
	) override;

private:
	void checkParameters(RichParameterList const&, vcg::CallBackPos&);
	void updateBorderFlags(CMeshO&, vcg::CallBackPos&);
	void checkSelectedMesh(CMeshO const&, vcg::CallBackPos&);
};

#endif
