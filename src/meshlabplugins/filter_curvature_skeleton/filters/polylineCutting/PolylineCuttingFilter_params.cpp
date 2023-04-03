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

#include "PolylineCuttingFilter.h"

#include "common/PolylineMesh.h"

 //defaults
#define DEFAULT_FACETAG "segmentation_tag"
#define DEFAULT_CLOSEHOLE_ADJ_FACETAG "hole_adj_segmentation_tag"

// displayed strings
#define POLYLINE_REFINE_CATEGORY "(0) Polyline Refinement"
#define REFINE_DO_DISPLAYNAME "Refine Polylines"
#define REFINE_DO_DESCRIPTION "Refine Polylines"
#define REFINE_ITERATION_DISPLAYNAME "Iterations"
#define REFINE_ITERATION_DESCRIPTION "Number of refinement iterations."
#define REFINE_SMOOTHW_DISPLAYNAME "Smooth Weight"
#define REFINE_SMOOTHW_DESCRIPTION "Smooth Weight"
#define REFINE_PROJECTW_DISPLAYNAME "Project Weight"
#define REFINE_PROJECTW_DESCRIPTION "Project Weight"
#define REFINE_FIT_PLANEW_DISPLAYNAME "Fit Plane Weight"
#define REFINE_FIT_PLANEW_DESCRIPTION "Fit Plane Weight"
#define REFINE_SEPARATION_DIST_DISPLAYNAME "Polyline Separation min distance"
#define REFINE_SEPARATION_DIST_DESCRIPTION "Polyline Separation min distance (% of bbox diag)"
#define REFINE_SEPARATIONW_DISPLAYNAME "Polyline Separation Weight"
#define REFINE_SEPARATIONW_DESCRIPTION "Polyline Separation Weight"

#define CUTTING_CATEGORY "(1) Polyline Cutting"
#define CLOSE_HOLES_DISPLAYNAME "Close Holes"
#define CLOSE_HOLES_DESCRIPTION "Close Holes in generated pieces."
#define REFINE_HOLE_EDGE_LENGHT_DISPLAYNAME "Refine Holes Edge Lenght"
#define REFINE_HOLE_EDGE_LENGHT_DESCRIPTION "Refine Holes Edge Lenght (% of bbox diag)"
#define CLOSE_HOLES_ADJACENCY_TAG_ID_DISPLAYNAME "Close Hole Face Tag attribute name"
#define CLOSE_HOLES_ADJACENCY_TAG_ID_DESCRIPTION "Close Hole Face Tag attribute name"

#define EXTRA_CATEGORY "(2) Extra"
#define FACE_TAG_ID_DISPLAYNAME "Face Tag attribute name"
#define FACE_TAG_ID_DESCRIPTION "Face Tag attribute name"
#define GENERATE_POLYLINES_DISPLAYNAME "Generate Polylines"
#define GENERATE_POLYLINES_DESCRIPTION "Generate Polylines"

namespace curvatureSkeleton
{

RichParameterList PolylineCuttingFilter::initParameterList(FilterPlugin const& p, MeshDocument const& m)
{
	RichParameterList parlst;

	auto* mesh_mm = m.mm();
	auto& mesh = mesh_mm->cm;

	Scalarm avg_length = 0;
	for (auto& face : mesh.face)
	{
		if (face.IsD()) continue;
		avg_length +=
			vcg::Distance(face.V(0)->cP(), face.V(1)->cP()) +
			vcg::Distance(face.V(1)->cP(), face.V(2)->cP()) +
			vcg::Distance(face.V(2)->cP(), face.V(0)->cP());
	}

	if(mesh.FN() > 0)
		avg_length /= (mesh.FN() * 3);

	parlst.addParam(RichString(PARAM_FACE_TAG_ID, DEFAULT_FACETAG, FACE_TAG_ID_DISPLAYNAME, FACE_TAG_ID_DESCRIPTION, true, EXTRA_CATEGORY));
	parlst.addParam(RichBool(PARAM_GENERATE_POLYLINES, false, GENERATE_POLYLINES_DISPLAYNAME, GENERATE_POLYLINES_DESCRIPTION, true, EXTRA_CATEGORY));

	parlst.addParam(RichBool(PARAM_DO_REFINE_POLYLINES, true, REFINE_DO_DISPLAYNAME, REFINE_DO_DESCRIPTION, false, POLYLINE_REFINE_CATEGORY));
	parlst.addParam(RichInt(PARAM_REFINE_ITERATIONS, 100, REFINE_ITERATION_DISPLAYNAME, REFINE_ITERATION_DESCRIPTION, false, POLYLINE_REFINE_CATEGORY));
	parlst.addParam(RichDynamicFloat(PARAM_REFINE_SMOOTH_WEIGTH, 0.6f, 0.f, 1.f, REFINE_SMOOTHW_DISPLAYNAME, REFINE_SMOOTHW_DESCRIPTION, false, POLYLINE_REFINE_CATEGORY));
	parlst.addParam(RichDynamicFloat(PARAM_REFINE_PROJECT_WEIGTH, 0.5f, 0.f, 1.f, REFINE_PROJECTW_DISPLAYNAME, REFINE_PROJECTW_DESCRIPTION, false, POLYLINE_REFINE_CATEGORY));
	parlst.addParam(RichDynamicFloat(PARAM_REFINE_FIT_PLANE_WEIGTH, 0.3f, 0.f, 1.f, REFINE_FIT_PLANEW_DISPLAYNAME, REFINE_FIT_PLANEW_DESCRIPTION, false, POLYLINE_REFINE_CATEGORY));
	parlst.addParam(RichDynamicFloat(PARAM_REFINE_SEPARATION_WEIGHT, 1.f, 0.f, 1.f, REFINE_SEPARATIONW_DISPLAYNAME, REFINE_SEPARATIONW_DESCRIPTION, false, POLYLINE_REFINE_CATEGORY));
	parlst.addParam(RichPercentage(PARAM_REFINE_SEPARATION_MIN_DISTANCE, 2 * avg_length, 0.f, mesh.bbox.Diag(), REFINE_SEPARATION_DIST_DISPLAYNAME, REFINE_SEPARATION_DIST_DESCRIPTION, false, POLYLINE_REFINE_CATEGORY));

	parlst.addParam(RichBool(PARAM_CLOSE_HOLES, true, CLOSE_HOLES_DISPLAYNAME, CLOSE_HOLES_DESCRIPTION, false, CUTTING_CATEGORY));
	parlst.addParam(RichString(PARAM_HOLE_ADJ_TAG_ID, DEFAULT_CLOSEHOLE_ADJ_FACETAG, CLOSE_HOLES_ADJACENCY_TAG_ID_DISPLAYNAME, CLOSE_HOLES_ADJACENCY_TAG_ID_DESCRIPTION, true, CUTTING_CATEGORY));
	parlst.addParam(RichPercentage(PARAM_REFINE_HOLE_EDGE_LENGHT, avg_length, 0.f, mesh.bbox.Diag(), REFINE_HOLE_EDGE_LENGHT_DISPLAYNAME, REFINE_HOLE_EDGE_LENGHT_DESCRIPTION, false, CUTTING_CATEGORY));

	return parlst;
}

}
