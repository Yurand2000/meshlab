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

#define FILTER_NAME "FilterCurvatureSkeleton"
#define FILTER_DISPLAYNAME "Curvature Skeleton"
#define FILTER_DISPLAYDESCRIPTION "No Description..."
#define FILTER_CLASS FilterPlugin::Other
#define FILTER_PYTHON_NAME "apply_curvature_skeleton"

/**
 * @brief Constructor usually performs only two simple tasks of filling the two lists
 *  - typeList: with all the possible id of the filtering actions
 *  - actionList with the corresponding actions. If you want to add icons to
 *  your filtering actions you can do here by construction the QActions accordingly
 */
FilterCurvatureSkeleton::FilterCurvatureSkeleton()
{ 
	typeList = {CURVATURE_SKELETON};

	for(ActionIDType tt : types())
		actionList.push_back(new QAction(filterName(tt), this));
}

FilterCurvatureSkeleton::~FilterCurvatureSkeleton()
{
}

QString FilterCurvatureSkeleton::pluginName() const
{
	return FILTER_NAME;
}

/**
 * @brief ST() must return the very short string describing each filtering action
 * (this string is used also to define the menu entry)
 * @param filterId: the id of the filter
 * @return the name of the filter
 */
QString FilterCurvatureSkeleton::filterName(ActionIDType filterId) const
{
	switch(filterId) {
	case CURVATURE_SKELETON :
		return FILTER_DISPLAYNAME;
	default :
		assert(0);
		return QString();
	}
}

/**
 * @brief FilterCurvatureSkeleton::pythonFilterName if you want that your filter should have a different
 * name on pymeshlab, use this function to return its python name.
 * @param f
 * @return
 */
QString FilterCurvatureSkeleton::pythonFilterName(ActionIDType f) const
{
	switch(f) {
	case CURVATURE_SKELETON :
		return FILTER_PYTHON_NAME;
	default :
		assert(0);
		return QString();
	}
}


/**
 * @brief // Info() must return the longer string describing each filtering action
 * (this string is used in the About plugin dialog)
 * @param filterId: the id of the filter
 * @return an info string of the filter
 */
 QString FilterCurvatureSkeleton::filterInfo(ActionIDType filterId) const
{
	switch(filterId) {
	case CURVATURE_SKELETON :
		return FILTER_DISPLAYDESCRIPTION;
	default :
		assert(0);
		return "Unknown Filter";
	}
}

 /**
 * @brief The FilterClass describes in which generic class of filters it fits.
 * This choice affect the submenu in which each filter will be placed
 * More than a single class can be chosen.
 * @param a: the action of the filter
 * @return the class od the filter
 */
FilterCurvatureSkeleton::FilterClass FilterCurvatureSkeleton::getClass(const QAction *a) const
{
	switch(ID(a)) {
	case CURVATURE_SKELETON :
		return FILTER_CLASS;
	default :
		assert(0);
		return FilterPlugin::Generic;
	}
}

/**
 * @brief FilterCurvatureSkeleton::filterArity
 * @return
 */
FilterPlugin::FilterArity FilterCurvatureSkeleton::filterArity(const QAction*) const
{
	return SINGLE_MESH;
}

/**
 * @brief FilterCurvatureSkeleton::getPreConditions
 * @return
 */
int FilterCurvatureSkeleton::getPreConditions(const QAction*) const
{
	return MeshModel::MM_NONE;
}

/**
 * @brief FilterCurvatureSkeleton::postCondition
 * @return
 */
int FilterCurvatureSkeleton::postCondition(const QAction*) const
{
	return MeshModel::MM_VERTCOORD;
}

/**
 * @brief This function define the needed parameters for each filter. Return true if the filter has some parameters
 * it is called every time, so you can set the default value of parameters according to the mesh
 * For each parameter you need to define,
 * - the name of the parameter,
 * - the default value
 * - the string shown in the dialog
 * - a possibly long string describing the meaning of that parameter (shown as a popup help in the dialog)
 * @param action
 * @param m
 * @param parlst
 */
RichParameterList FilterCurvatureSkeleton::initParameterList(const QAction *action,const MeshModel &m)
{
	RichParameterList parlst;
	switch(ID(action)) {
	case CURVATURE_SKELETON :
		//parlst.addParam(RichOpenFile("ModelFile", ".", {"*.obj"}, "Model file to skeletonize", "No Tooltip..."));
		//parlst.addParam(RichString("ModelFile", "C:/Users/Yuri/Documents/gitDatabase/meshlab/src/meshlabplugins/filter_curvature_skeleton/example.off", "Model File", "No Tooltip..."));
		break;
	default :
		assert(0);
	}
	return parlst;
}

std::map<std::string, QVariant> applyCurveSkeleton(MeshDocument& md);

/**
 * @brief The Real Core Function doing the actual mesh processing.
 * @param action
 * @param md: an object containing all the meshes and rasters of MeshLab
 * @param par: the set of parameters of each filter
 * @param cb: callback object to tell MeshLab the percentage of execution of the filter
 * @return true if the filter has been applied correctly, false otherwise
 */
std::map<std::string, QVariant> FilterCurvatureSkeleton::applyFilter(const QAction * action, const RichParameterList & parameters, MeshDocument &md, unsigned int&, vcg::CallBackPos *cb)
{
	switch(ID(action)) {
	case CURVATURE_SKELETON:
		{
			return applyCurveSkeleton( md );
		}
	default :
		wrongActionCalled(action);
	}
	return std::map<std::string, QVariant>();
}

MESHLAB_PLUGIN_NAME_EXPORTER(FilterCurvatureSkeleton)

#include "cgal_adapter/mesh_skeletonizer.h"

std::map<std::string, QVariant> applyCurveSkeleton(MeshDocument& md)
{
	auto current_mesh = md.mm()->cm;
	auto skeletonizer = CGalAdapter::MeshSkeletonizer(current_mesh);

	skeletonizer.computeStep();
	//auto skeleton = skeletonizer.getSkeleton();
	auto mesoSkeleton = skeletonizer.getMesoSkeleton();

	//md.addNewMesh(skeleton, "skeleton");
	md.addNewMesh(mesoSkeleton, "mesoSkeleton");

	return std::map<std::string, QVariant>();
}
