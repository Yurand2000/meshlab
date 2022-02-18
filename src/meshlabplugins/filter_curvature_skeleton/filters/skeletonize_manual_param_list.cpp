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

#include "skeletonize_manual.h"

#define _USE_MATH_DEFINES
#include <cmath>

// default values
#define MAX_ITERATIONS_DEFAULT 100
#define DELTA_AREA_DEFAULT Scalarm(0.0001)
#define MAX_TRIANGLE_ANGLE_DEFAULT Scalarm(110 * (M_PI / 180.0))
//#define MIN_EDGE_LENGTH_DEFAULT Scalarm(-1)
#define QUALITY_TRADEOFF_DEFAULT Scalarm(0.1)
#define ENABLE_MEDIALLY_CENTERING_DEFAULT true
#define MEDIALLY_CENTERING_TRADEOFF_DEFAULT Scalarm(0.2)

// displayed strings
#define GENERIC_CATEGORY "Algorithm Termination Options"
#define MAX_ITERATIONS_DISPLAYNAME   "Max Iterations"
#define MAX_ITERATIONS_DESCRIPTION   "Upper limit on the iterations of the algorithm. The algorithm will stop if no more skeletoning can be achived anyways."
#define DELTA_AREA_TERMINATION_DISPLAYNAME "Area Variation Factor"
#define DELTA_AREA_TERMINATION_DESCRIPTION "The convergence is considered to be reached if the variation of the area of the meso-skeleton after one iteration is smaller than the area variation factor times the original area of the mesh. Must be positive."
#define GENERATE_INTERMEDIATE_MESHES_DISPLAYNAME "Generate Intermediate Meshes"
#define GENERATE_INTERMEDIATE_MESHES_DESCRIPTION "Generate the intermediate meshes of the algorithm."

#define LOCAL_REMESHING_CATEGORY "Local Remeshing"
#define MAX_ANGLE_DISPLAYNAME "Max angle"
#define MAX_ANGLE_DESCRIPTION "During remeshing steps, a triangle will be split if it has an angle larger than the given angle. Angle in radians. Must be bigger than PI/2 (~1.57)."
#define MIN_EDGE_LENGTH_DISPLAYNAME "Min length"
#define MIN_EDGE_LENGTH_DESCRIPTION "During remeshing steps, an edge will be collapse if it is length is less than the given value. Length in application units. Must be positive."

#define VERTEX_MOTION_OPTIONS_CATEGORY "Vertex Motion Options"
#define QUALITY_SPEED_TRADEOFF_DISPLAYNAME "Quality-Speed Tradeoff value"
#define QUALITY_SPEED_TRADEOFF_DESCRIPTION "*ADVANCED* Controls the velocity of movement and approximation quality: decreasing this value makes the mean curvature flow based contraction converge faster, but results in a skeleton of lower quality. Must be positive."
#define USE_MEDIALLY_CENTERING_DISPLAYNAME "Enable Medially Centering"
#define USE_MEDIALLY_CENTERING_DESCRIPTION "If true, the meso-skeleton placement will be attracted by an approximation of the medial axis of the mesh during the contraction steps, so will be the result skeleton."
#define MEDIALLY_CENTERING_VALUE_DISPLAYNAME "Medially Centering Tradeoff value"
#define MEDIALLY_CENTERING_VALUE_DESCRIPTION "*ADVANCED* Controls the smoothness of the medial approximation: increasing this value results in a (less smooth) skeleton closer to the medial axis, as well as a lower convergence speed. Must be positive."

Scalarm calculateMinEdgeLength(CMeshO const& mesh);

RichParameterList filterSkeletonizeManual::initParameterList(FilterPlugin const&, MeshModel const& m)
{
	RichParameterList parlst;
	parlst.addParam(RichInt(PARAM_MAX_ITERATIONS, MAX_ITERATIONS_DEFAULT, MAX_ITERATIONS_DISPLAYNAME, MAX_ITERATIONS_DESCRIPTION, false, GENERIC_CATEGORY));
	parlst.addParam(RichFloat(PARAM_DELTA_AREA_TERMINATION, DELTA_AREA_DEFAULT, DELTA_AREA_TERMINATION_DISPLAYNAME, DELTA_AREA_TERMINATION_DESCRIPTION, false, GENERIC_CATEGORY));
	parlst.addParam(RichBool(PARAM_GENERATE_INTERMEDIATE_MESHES, false, GENERATE_INTERMEDIATE_MESHES_DISPLAYNAME, GENERATE_INTERMEDIATE_MESHES_DESCRIPTION, false, GENERIC_CATEGORY));

	parlst.addParam(RichFloat(PARAM_MAX_ANGLE, MAX_TRIANGLE_ANGLE_DEFAULT, MAX_ANGLE_DISPLAYNAME, MAX_ANGLE_DESCRIPTION, false, LOCAL_REMESHING_CATEGORY));
	parlst.addParam(RichFloat(PARAM_MIN_EDGE_LENGTH, calculateMinEdgeLength(m.cm), MIN_EDGE_LENGTH_DISPLAYNAME, MIN_EDGE_LENGTH_DESCRIPTION, false, LOCAL_REMESHING_CATEGORY));

	parlst.addParam(RichFloat(PARAM_QUALITY_TRADEOFF, QUALITY_TRADEOFF_DEFAULT, QUALITY_SPEED_TRADEOFF_DISPLAYNAME, QUALITY_SPEED_TRADEOFF_DESCRIPTION, false, VERTEX_MOTION_OPTIONS_CATEGORY));
	parlst.addParam(RichBool(PARAM_ENABLE_MEDIALLY_CENTERING, ENABLE_MEDIALLY_CENTERING_DEFAULT, USE_MEDIALLY_CENTERING_DISPLAYNAME, USE_MEDIALLY_CENTERING_DESCRIPTION, false, VERTEX_MOTION_OPTIONS_CATEGORY));
	parlst.addParam(RichFloat(PARAM_MEDIALLY_CENTERING_TRADEOFF, MEDIALLY_CENTERING_TRADEOFF_DEFAULT, MEDIALLY_CENTERING_VALUE_DISPLAYNAME, MEDIALLY_CENTERING_VALUE_DESCRIPTION, false, VERTEX_MOTION_OPTIONS_CATEGORY));
	return parlst;
}

Scalarm calculateMinEdgeLength(CMeshO const& mesh)
{
	auto boundingBox = mesh.trBB();
	return 0.002 * boundingBox.Diag();
}
