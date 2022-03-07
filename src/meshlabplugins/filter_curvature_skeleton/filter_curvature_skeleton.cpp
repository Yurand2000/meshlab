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

#include "filter_curvature_skeleton.h"

#include "filters/skeletonize_manual.h"
#include "filters/skeleton_index_to_quality.h"
#include "filters/skeleton_distance_to_quality.h"

#define PLUGIN_NAME "FilterCurvatureSkeleton"

FilterCurvatureSkeleton::FilterCurvatureSkeleton()
{
	filters = {};
	filters.push_back(new filterSkeletonizeManual());
	filters.push_back(new filterSkeletonIndexToMeshQuality());
	filters.push_back(new filterSkeletonDistanceToMeshQuality());

	typeList = { SKELETONIZE_MANUAL, SKELETON_INDEX_TO_MESH_QUALITY, SKELETON_DISTANCE_TO_MESH_QUALITY };

	for(ActionIDType tt : types())
		actionList.push_back( new QAction(filterName(tt), this) );
}

FilterCurvatureSkeleton::~FilterCurvatureSkeleton()
{
	for (auto filter : filters)
	{
		delete filter;
	}
}

QString FilterCurvatureSkeleton::pluginName() const
{
	return PLUGIN_NAME;
}

QString FilterCurvatureSkeleton::filterName(ActionIDType filterId) const
{
	return findFilterAndExecute<QString>(filterId,
		[](templateFilter const& filter) { return filter.filterName(); },
		[](){ assert(0); return QString(); });
}

QString FilterCurvatureSkeleton::pythonFilterName(ActionIDType filterId) const
{
	return findFilterAndExecute<QString>(filterId,
		[](templateFilter const& filter) { return filter.pythonFilterName(); },
		[](){ assert(0); return QString(); });
}

 QString FilterCurvatureSkeleton::filterInfo(ActionIDType filterId) const
{
	return findFilterAndExecute<QString>(filterId,
		[](templateFilter const& filter) { return filter.filterInfo(); },
		[]() { assert(0); return QString("Unknown Filter"); });
}

FilterCurvatureSkeleton::FilterClass FilterCurvatureSkeleton::getClass(const QAction *a) const
{
	return findFilterAndExecute<FilterCurvatureSkeleton::FilterClass>(a,
		[](templateFilter const& filter) { return filter.getClass(); },
		[]() { assert(0); return FilterPlugin::Generic; });
}

FilterPlugin::FilterArity FilterCurvatureSkeleton::filterArity(const QAction* a) const
{
	return findFilterAndExecute<FilterCurvatureSkeleton::FilterArity>(a,
		[](templateFilter const& filter) { return filter.filterArity(); },
		[]() { assert(0); return FilterPlugin::FilterArity::NONE; });
}

int FilterCurvatureSkeleton::getPreConditions(const QAction* a) const
{
	return findFilterAndExecute<int>(a,
		[](templateFilter const& filter) { return filter.getPreConditions(); },
		[]() { assert(0); return MeshModel::MM_NONE; });
}

int FilterCurvatureSkeleton::postCondition(const QAction* a) const
{
	return findFilterAndExecute<int>(a,
		[](templateFilter const& filter) { return filter.postCondition(); },
		[]() { assert(0); return MeshModel::MM_NONE; });
}

RichParameterList FilterCurvatureSkeleton::initParameterList(QAction const* a, MeshModel const& model)
{
	return findFilterAndExecute<RichParameterList>(a,
		[this, &model](templateFilter& filter) { return filter.initParameterList(*this, model); },
		[]() { assert(0); return RichParameterList(); });
}

std::map<std::string, QVariant> FilterCurvatureSkeleton::applyFilter(
	QAction const* a,
	RichParameterList const& params,
	MeshDocument& document,
	unsigned int& val,
	vcg::CallBackPos* callback)
{
	return findFilterAndExecute< std::map<std::string, QVariant> >(a,
		[this, &params, &document, &val, callback](templateFilter& filter)
		{
			return filter.applyFilter(*this, params, document, val, callback);
		},

		[a]() { wrongActionCalled(a); return std::map<std::string, QVariant>(); }
	);
}

MESHLAB_PLUGIN_NAME_EXPORTER(FilterCurvatureSkeleton)
