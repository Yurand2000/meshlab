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

#ifndef FILTERCURVATURESKELETON_CGAL_TYPEDEFS
#define FILTERCURVATURESKELETON_CGAL_TYPEDEFS

#include <CGAL/Mean_curvature_flow_skeletonization.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>

namespace curvatureSkeleton { namespace CGalAdapter
{
	typedef CGAL::Simple_cartesian<double>                      CGALKernel;
	typedef CGALKernel::Point_3									CGALPoint;
	typedef CGAL::Surface_mesh<CGALPoint>                       CGALMesh;
	typedef CGAL::SM_Vertex_index							    CGALVertexIndex;
	typedef CGAL::Mean_curvature_flow_skeletonization<CGALMesh> CGALSkeletonizer;
	typedef CGALSkeletonizer::Skeleton                          CGALSkeleton;
	typedef CGALSkeletonizer::Meso_skeleton                     CGALMesoSkeleton;
} }

#endif //FILTERCURVATURESKELETON_CGAL_TYPEDEFS
