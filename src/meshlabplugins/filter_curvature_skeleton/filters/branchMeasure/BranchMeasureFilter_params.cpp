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

#include "BranchMeasureFilter.h"

#define MEASURE_ONLY_SELECTED_DISPLAYNAME "Measure Only Selected"
#define MEASURE_ONLY_SELECTED_DESCRIPTION "Measure only the selected mesh instead of all the visible meshes."
#define GENERATE_SKELETONS_DISPLAYNAME "Generate Skeletons"
#define GENERATE_SKELETONS_DESCRIPTION "Save generated skeleton meshes."
#define SAVE_ON_FILE_DISPLAYNAME "Save measures on file"
#define SAVE_ON_FILE_DESCRIPTION "Save generated measures on file."

namespace curvatureSkeleton
{

RichParameterList BranchMeasureFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;


	parlst.addParam(RichBool(PARAM_MEASURE_ONLY_SELECTED, false, MEASURE_ONLY_SELECTED_DISPLAYNAME, MEASURE_ONLY_SELECTED_DESCRIPTION));
	parlst.addParam(RichBool(PARAM_SAVE_SKELETONS, false, GENERATE_SKELETONS_DISPLAYNAME, GENERATE_SKELETONS_DESCRIPTION));
	parlst.addParam(RichFileSave(PARAM_SAVE_FILE, "", ".csv", SAVE_ON_FILE_DISPLAYNAME, SAVE_ON_FILE_DESCRIPTION));

	return parlst;
}

}
