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

#ifndef FILTERCURVATURESKELETON_CGAL_MESH_CONVERTER
#define FILTERCURVATURESKELETON_CGAL_MESH_CONVERTER

#include "typedefs.h"
#include <common/ml_document/mesh_document.h>

namespace curvatureSkeleton { namespace CGalAdapter
{
	CGALMesh convertCMeshToCGALMesh(CMeshO const& mesh);

	CMeshO convertCGALMesoSkeletonToCMesh(CGALMesoSkeleton const& meso_skeleton);
	CMeshO convertCGALSkeletonToCMesh(CGALSkeleton const& skeleton);
} }

#endif //FILTERCURVATURESKELETON_CGAL_MESH_CONVERTER
