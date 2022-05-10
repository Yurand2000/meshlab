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

#ifndef FILTERCURVATURESKELETON_SIMPLIFY_SKELETON
#define FILTERCURVATURESKELETON_SIMPLIFY_SKELETON

#include "SkeletonMesh.h"

namespace curvatureSkeleton
{

class SimplifySkeleton
{
public:
	static bool isMeshConnected(SkeletonMesh const& skeleton);
	static void collapseTwoConnectedVertices(SkeletonMesh& skeleton);
	static void collapseShortEdges(SkeletonMesh& skeleton, int root_node, Scalarm min_length);

private:
	~SimplifySkeleton() = delete;
};

}

#endif // FILTERCURVATURESKELETON_SIMPLIFY_SKELETON
