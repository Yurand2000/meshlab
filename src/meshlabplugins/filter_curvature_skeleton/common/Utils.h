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

#ifndef FILTERCURVATURESKELETON_UTILITY_FUNCTIONS
#define FILTERCURVATURESKELETON_UTILITY_FUNCTIONS

#include <common/ml_document/cmesh.h>
#include <vector>

namespace curvatureSkeleton
{
    template <typename MESH>
    class Utils
    {
    public:
        static typename MESH::VertexType const* getVertexInMesh(vcg::Point3<Scalarm> point, MESH const& mesh);
        static typename MESH::VertexType* getVertexInMesh(vcg::Point3<Scalarm> point, MESH& mesh);
        static int getVertexIndexInMesh(vcg::Point3<Scalarm> point, MESH const& mesh);
    };
}

#include "Utils.imp.h"

#endif // FILTERCURVATURESKELETON_UTILITY_FUNCTIONS
