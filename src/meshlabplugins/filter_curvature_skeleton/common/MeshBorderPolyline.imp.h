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

#ifndef FILTERCURVATURESKELETON_MESH_BORDER_POLYLINE_IMP
#define FILTERCURVATURESKELETON_MESH_BORDER_POLYLINE_IMP

#include <vector>
#include <queue>
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/outline_support.h>
#include <common/ml_document/cmesh.h>
#include <common/mlexception.h>

namespace curvatureSkeleton
{
	namespace detail
	{
		static Scalarm getPolylineLength(std::vector<Point3m> const& polyline);
		static int getLongestPolylineIndex(std::vector< std::vector<Point3m> > const& polylines);

		template<typename MESH> static void polylineToMesh(MESH& mesh, std::vector<Point3m> const& polyline);
	}

	template<typename MESH>
	std::vector<MESH> MeshBorderPolyline<MESH>::getPolylines(MESH& mesh)
	{
		std::vector<MESH> polylines;

		std::vector< std::vector<Point3m> > outlines;
		vcg::tri::OutlineUtil<Scalarm>::ConvertMeshBoundaryToOutline3Vec(mesh, outlines);

		for (auto& outline : outlines)
		{
			polylines.emplace_back();
			detail::polylineToMesh(polylines.back(), outline);
		}

		return polylines;
	}

	template<typename MESH>
	MESH MeshBorderPolyline<MESH>::getLongestPolyline(MESH& mesh)
	{
		MESH polyline;

		std::vector< std::vector<Point3m> > outlines;
		vcg::tri::OutlineUtil<Scalarm>::ConvertMeshBoundaryToOutline3Vec(mesh, outlines);
		if (!outlines.empty())
		{
			int longest_polyline_index = detail::getLongestPolylineIndex(outlines);
			detail::polylineToMesh(polyline, outlines[longest_polyline_index]);
		}

		return polyline;
	}

	Scalarm detail::getPolylineLength(std::vector<Point3m> const& polyline)
	{
		Scalarm len = 0;
		for (int i = 0; i < polyline.size(); i++)
		{
			len += vcg::Distance(polyline[i], polyline[(i + 1) % polyline.size()]);
		}

		return len;
	}

	int detail::getLongestPolylineIndex(std::vector< std::vector<Point3m> > const& polylines)
	{
		Scalarm max = 0;
		int max_index = 0;
		for (int i = 0; i < polylines.size(); i++)
		{
			auto& polyline = polylines[i];
			auto len = detail::getPolylineLength(polyline);

			if (len > max)
			{
				max = len;
				max_index = i;
			}
		}

		return max_index;
	}

	template<typename MESH> void detail::polylineToMesh(MESH& mesh, std::vector<Point3m> const& polyline)
	{
		std::vector< std::vector<Point3m> > temp_polylines;
		temp_polylines.push_back(polyline);

		vcg::tri::OutlineUtil<Scalarm>::ConvertOutline3VecToEdgeMesh(temp_polylines, mesh);
	}
}

#endif // FILTERCURVATURESKELETON_MESH_BORDER_POLYLINE_IMP
