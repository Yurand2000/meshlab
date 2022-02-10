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
	enum {
		CURVATURE_SKELETON
	};

	FilterCurvatureSkeleton();
	virtual ~FilterCurvatureSkeleton();

	QString pluginName() const;

	QString filterName(ActionIDType filter) const;
	QString pythonFilterName(ActionIDType f) const;
	QString filterInfo(ActionIDType filter) const;
	FilterClass getClass(const QAction* a) const;
	FilterArity filterArity(const QAction*) const;
	int getPreConditions(const QAction *) const;
	int postCondition(const QAction* ) const;
	RichParameterList initParameterList(const QAction*, const MeshModel &);
	std::map<std::string, QVariant> applyFilter(
		const QAction* action,
		const RichParameterList & parameters,
		MeshDocument &md,
		unsigned int& postConditionMask,
		vcg::CallBackPos * cb);

private:
	void checkParameters(RichParameterList const&, vcg::CallBackPos&);
	void updateBorderFlags(CMeshO&, vcg::CallBackPos&);
	void checkSelectedMesh(CMeshO const&, vcg::CallBackPos&);
	std::map<std::string, QVariant> applyAlgorithm(CMeshO const&, MeshDocument&, RichParameterList const&, vcg::CallBackPos&);
};

#endif
