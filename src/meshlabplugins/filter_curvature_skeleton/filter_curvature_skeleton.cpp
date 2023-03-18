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

#include "filters/skeletonize/SkeletonizeFilter.h"
#include "filters/pruneSkeleton/PruneSkeletonFilter.h"
#include "filters/treeSegmentation/TreeSegmentationFilter.h"
#include "filters/polylineCutting/PolylineCuttingFilter.h"

#include "filters/pruneExtend/PruneExtendFilter.h"
#include "filters/orderCompute/OrderComputeFilter.h"

#ifdef CURVATURE_SKELETON_TEST
#include "test_filters/testFilters.h"
#include "test_filters/findPath/FindPathFilter.h"
#include "test_filters/findPathFill/FindPathFillFilter.h"
#include "test_filters/computeBranches/ComputeBranchesFilter.h"
#include "test_filters/computePolylines/ComputePolylinesFilter.h"
#include "test_filters/extractPolyline/ExtractPolylineFilter.h"
#include "test_filters/refinePolyline/RefinePolylineFilter.h"
#include "test_filters/cutOnPolylines/CutOnPolylinesFilter.h"
#include "test_filters/showMinRadius/ShowMinRadiusFilter.h"
#include "test_filters/showPolylineParent/ShowPolylineParentFilter.h"
#include "test_filters/closeHoles/CloseHolesFilter.h"
#include "test_filters/measureBranch/MeasureBranchFilter.h"
#endif

#define PLUGIN_NAME "FilterCurvatureSkeleton"

namespace curvatureSkeleton
{

FilterCurvatureSkeleton::FilterCurvatureSkeleton()
{
	filters.push_back( std::make_unique<SkeletonizeFilter>() );
	filters.push_back( std::make_unique<PruneSkeletonFilter>() );
	filters.push_back( std::make_unique<TreeSegmentationFilter>() );
	filters.push_back( std::make_unique<PolylineCuttingFilter>() );

	filters.push_back( std::make_unique<PruneExtendFilter>() );
	filters.push_back( std::make_unique<OrderComputeFilter>() );

	typeList = {
		SKELETONIZE,
		PRUNE_SKELETON,
		TREE_SEGMENTATION,
		POLYLINE_CUTTING,

		PRUNE_AND_EXTEND,
		COMPUTE_ORDER_NUMBERS,
	};

#ifdef CURVATURE_SKELETON_TEST
	filters.push_back( std::make_unique<FindPathTestFilter>() );
	filters.push_back( std::make_unique<FindPathFillTestFilter>() );
	filters.push_back( std::make_unique<ComputeBranchesTestFilter>() );
	filters.push_back( std::make_unique<ExtractPolylineTestFilter>() );
	filters.push_back( std::make_unique<RefinePolylineTestFilter>() );
	filters.push_back( std::make_unique<ComputePolylinesTestFilter>() );
	filters.push_back( std::make_unique<CutOnPolylinesTestFilter>() );
	filters.push_back( std::make_unique<ShowMinRadiusTestFilter>() );
	filters.push_back( std::make_unique<ShowPolylineParentTestFilter>() );
	filters.push_back( std::make_unique<CloseHolesTestFilter>() );
	filters.push_back( std::make_unique<MeasureBranchTestFilter>() );
	typeList.push_back(TestFilters::FIND_PATH);
	typeList.push_back(TestFilters::FIND_PATH_FILL);
	typeList.push_back(TestFilters::COMPUTE_BRANCHES);
	typeList.push_back(TestFilters::COMPUTE_POLYLINES);
	typeList.push_back(TestFilters::EXTRACT_POLYLINE);
	typeList.push_back(TestFilters::REFINE_POLYLINE);
	typeList.push_back(TestFilters::CUT_ON_POLYLINES);
	typeList.push_back(TestFilters::SHOW_MIN_RADIUS);
	typeList.push_back(TestFilters::SHOW_POLYLINE_PARENT);
	typeList.push_back(TestFilters::CLOSE_HOLES);
	typeList.push_back(TestFilters::MEASURE_BRANCH);
#endif

	for(ActionIDType tt : types())
		actionList.push_back( new QAction(filterName(tt), this) );
}

QString FilterCurvatureSkeleton::pluginName() const
{
	return PLUGIN_NAME;
}

QString FilterCurvatureSkeleton::filterName(ActionIDType filterId) const
{
	return findFilterAndExecute<QString>(filterId,
		[](TemplateFilter const& filter) { return filter.filterName(); },
		[](){ assert(0); return QString(); });
}

QString FilterCurvatureSkeleton::pythonFilterName(ActionIDType filterId) const
{
	return findFilterAndExecute<QString>(filterId,
		[](TemplateFilter const& filter) { return filter.pythonFilterName(); },
		[](){ assert(0); return QString(); });
}

 QString FilterCurvatureSkeleton::filterInfo(ActionIDType filterId) const
{
	return findFilterAndExecute<QString>(filterId,
		[](TemplateFilter const& filter) { return filter.filterInfo(); },
		[]() { assert(0); return QString("Unknown Filter"); });
}

FilterCurvatureSkeleton::FilterClass FilterCurvatureSkeleton::getClass(const QAction *a) const
{
	return findFilterAndExecute<FilterCurvatureSkeleton::FilterClass>(a,
		[](TemplateFilter const& filter) { return filter.getClass(); },
		[]() { assert(0); return FilterPlugin::Generic; });
}

FilterPlugin::FilterArity FilterCurvatureSkeleton::filterArity(const QAction* a) const
{
	return findFilterAndExecute<FilterCurvatureSkeleton::FilterArity>(a,
		[](TemplateFilter const& filter) { return filter.filterArity(); },
		[]() { assert(0); return FilterPlugin::FilterArity::NONE; });
}

int FilterCurvatureSkeleton::getPreConditions(const QAction* a) const
{
	return findFilterAndExecute<int>(a,
		[](TemplateFilter const& filter) { return filter.getPreConditions(); },
		[]() { assert(0); return MeshModel::MM_NONE; });
}

int FilterCurvatureSkeleton::postCondition(const QAction* a) const
{
	return findFilterAndExecute<int>(a,
		[](TemplateFilter const& filter) { return filter.postCondition(); },
		[]() { assert(0); return MeshModel::MM_NONE; });
}

RichParameterList FilterCurvatureSkeleton::initParameterList(QAction const* a, MeshDocument const& document)
{
	return findFilterAndExecute<RichParameterList>(a,
		[this, &document](TemplateFilter& filter) { return filter.initParameterList(*this, document); },
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
		[this, &params, &document, &val, callback](TemplateFilter& filter)
		{
			return filter.applyFilter(*this, params, document, val, callback);
		},

		[a]() { wrongActionCalled(a); return std::map<std::string, QVariant>(); }
	);
}

MESHLAB_PLUGIN_NAME_EXPORTER(FilterCurvatureSkeleton)

}
