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

#define PLUGIN_NAME "FilterCurvatureSkeleton"
#define F_SKELETONIZE_MANUAL_DISPLAYNAME "Skeletonize Mesh"
#define F_SKELETONIZE_MANUAL_DESCRIPTION "Manual Skeletonization of the selected mesh."
#define F_SKELETONIZE_MANUAL_CLASS		 FilterPlugin::Other
#define F_SKELETONIZE_MANUAL_PYTHON_NAME "skeletonizer_mesh"

FilterCurvatureSkeleton::FilterCurvatureSkeleton()
{ 
	typeList = { CURVATURE_SKELETON };

	for(ActionIDType tt : types())
		actionList.push_back( new QAction(filterName(tt), this) );
}

FilterCurvatureSkeleton::~FilterCurvatureSkeleton() { }

QString FilterCurvatureSkeleton::pluginName() const
{
	return PLUGIN_NAME;
}

QString FilterCurvatureSkeleton::filterName(ActionIDType filterId) const
{
	switch(filterId)
	{
	case CURVATURE_SKELETON :
		return F_SKELETONIZE_MANUAL_DISPLAYNAME;
	default :
		assert(0);
		return QString();
	}
}

QString FilterCurvatureSkeleton::pythonFilterName(ActionIDType f) const
{
	switch(f)
	{
	case CURVATURE_SKELETON :
		return F_SKELETONIZE_MANUAL_PYTHON_NAME;
	default :
		assert(0);
		return QString();
	}
}

 QString FilterCurvatureSkeleton::filterInfo(ActionIDType filterId) const
{
	switch(filterId) {
	case CURVATURE_SKELETON :
		return F_SKELETONIZE_MANUAL_DESCRIPTION;
	default :
		assert(0);
		return "Unknown Filter";
	}
}

FilterCurvatureSkeleton::FilterClass FilterCurvatureSkeleton::getClass(const QAction *a) const
{
	switch(ID(a)) {
	case CURVATURE_SKELETON :
		return F_SKELETONIZE_MANUAL_CLASS;
	default :
		assert(0);
		return FilterPlugin::Generic;
	}
}

FilterPlugin::FilterArity FilterCurvatureSkeleton::filterArity(const QAction* a) const
{
	switch (ID(a))
	{
	case CURVATURE_SKELETON :
		return SINGLE_MESH;
	default :
		assert(0);
		return NONE;
	}
}

int FilterCurvatureSkeleton::getPreConditions(const QAction* a) const
{
	switch (ID(a))
	{
	case CURVATURE_SKELETON :
		return MeshModel::MM_VERTCOORD | MeshModel::MM_FACEVERT;
	default :
		assert(0);
		return MeshModel::MM_NONE;
	}
}

int FilterCurvatureSkeleton::postCondition(const QAction* a) const
{
	switch (ID(a))
	{
	case CURVATURE_SKELETON :
		return MeshModel::MM_NONE;
	default :
		assert(0);
		return MeshModel::MM_NONE;
	}
}

