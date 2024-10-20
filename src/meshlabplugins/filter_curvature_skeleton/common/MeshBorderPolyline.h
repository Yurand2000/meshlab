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

#ifndef FILTERCURVATURESKELETON_MESH_BORDER_POLYLINE
#define FILTERCURVATURESKELETON_MESH_BORDER_POLYLINE

#include <common/ml_document/cmesh.h>
#include <vector>

namespace curvatureSkeleton
{
	template<typename MESH>
	class MeshBorderPolyline
	{
	public:
		static std::vector<MESH> getPolylines(MESH& mesh);
		static MESH getLongestPolyline(MESH& mesh);

	private:
		~MeshBorderPolyline() = delete;
	};

}

#include "MeshBorderPolyline.imp.h"

#endif // FILTERCURVATURESKELETON_MESH_BORDER_POLYLINE
