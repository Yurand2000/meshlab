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

#include "BranchExtender.h"

#include "additionalAttributeNames.h"
#include "SkeletonMesh.h"

#include <vector>
#include <common/plugins/interfaces/filter_plugin.h>
#include <vcg/complex/complex.h>
#include <vcg/complex/append.h>

namespace curvatureSkeleton
{

typedef vcg::Point3<Scalarm> Point;
typedef vcg::Point3<Scalarm> Normal;

typedef vcg::tri::UpdateTopology<SkeletonMesh> SkeletonMeshTopology;
typedef vcg::tri::Allocator<CMeshO> CMeshOAllocator;
typedef vcg::tri::Append<CMeshO, SkeletonMesh> SkeletonToCMeshOAppend;
typedef vcg::tri::Append<SkeletonMesh, CMeshO> CMeshOToSkeletonAppend;

struct SkeletonLeaf
{
	uint   index;
	Normal normal;
};

static std::vector<SkeletonLeaf> findSkeletonLeafs(SkeletonMesh const& skeleton);
static void extendBranch(SkeletonLeaf const& leaf, CMeshO const& mesh, CMeshO& skeleton, float angle);

void BranchExtender::extendLeafs(CMeshO const& mesh, CMeshO& skeleton, float angle)
{
	SkeletonMesh converted_skeleton;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_skeleton, skeleton);
	SkeletonMeshTopology::VertexEdge(converted_skeleton);
	auto vertices = findSkeletonLeafs(converted_skeleton);

	for (auto& leaf : vertices)
	{
		extendBranch(leaf, mesh, skeleton, angle);
	}
}

inline static bool   isLeafVertex(SkeletonVertex const& vertex);
inline static Normal getVertexNormal(SkeletonVertex const& vertex, int max_depth = 5);

std::vector<SkeletonLeaf> findSkeletonLeafs(SkeletonMesh const& skeleton)
{
	std::vector<SkeletonLeaf> leafs;
	for (uint i = 0; i < skeleton.vert.size(); i++)
	{
		auto& vertex = skeleton.vert[i];
		if (isLeafVertex(vertex))
		{
			leafs.push_back({
				i,
				getVertexNormal(vertex)
			});
		}
	}
	return leafs;
}

inline static bool isLeafVertex(SkeletonVertex const& vertex)
{
	return vcg::edge::VEDegree<SkeletonEdge>(&vertex) == 1;
}

static Normal getVertexNormalRecursive(SkeletonVertex const* vertex, SkeletonVertex const* parent, Normal curr_normal, int& depth);

inline static Normal getVertexNormal(SkeletonVertex const& vertex, int max_depth)
{
	int depth = max_depth;
	auto normal = getVertexNormalRecursive(&vertex, &vertex, Normal(0, 0, 0), depth);
	if (depth != max_depth)
	{
		normal.normalize();
		return normal;
	}
	else
		throw std::runtime_error("Normal could not be found");
}

static Normal getVertexNormalRecursive(SkeletonVertex const* vertex, SkeletonVertex const* parent, Normal curr_normal, int& depth)
{
	if (depth == 0)
		return curr_normal;

	std::vector<SkeletonVertex*> vertices;
	vcg::edge::VVStarVE(vertex, vertices);
	if (vertices.size() < 2)
	{
		auto next_vertex = vertices[0];
		if (vertices[0] == parent)
			next_vertex = vertices[1];

		auto this_normal = vertex->cP() - next_vertex->cP();

		curr_normal += this_normal;

		depth--;
		return getVertexNormalRecursive(next_vertex, vertex, curr_normal, depth);
	}
	else
		return curr_normal;
}




static std::vector<CVertexO const*> getMeshLeafVertices(uint vertex_index, CMeshO const& mesh);
static Point calculateBranchExtension(
	Point const&  leaf_vertex,
	std::vector<CVertexO const*> const& leaf_vertices,
	Normal const& leaf_normal,
	float         angle);

void extendBranch(SkeletonLeaf const& leaf, CMeshO const& mesh, CMeshO& skeleton, float angle)
{
	auto leaf_vertices = getMeshLeafVertices(leaf.index, mesh);
	auto new_point = calculateBranchExtension(skeleton.vert[leaf.index].cP(), leaf_vertices, leaf.normal, angle);
	if ( new_point != Point(0,0,0) )
	{
		CMeshOAllocator::AddVertex(skeleton, new_point);
		CMeshOAllocator::AddEdge(skeleton, leaf.index, (skeleton.VN() - 1));
	}
}

std::vector<CVertexO const*> getMeshLeafVertices(uint vertex_index, CMeshO const& mesh)
{
	std::vector<CVertexO const*> vertices;
	auto iterator = vcg::tri::Allocator<CMeshO>::FindPerVertexAttribute<uint>(
        mesh, ATTRIBUTE_MESH_TO_SKELETON_INDEX_NAME);

	if (vcg::tri::Allocator<CMeshO>::IsValidHandle(mesh, iterator))
	{
		for (uint i = 0; i < mesh.vert.size(); i++)
		{
			if (iterator[i] == vertex_index)
			{
				vertices.push_back( &(mesh.vert[i]) );
			}
		}
	}
	else
	{
		throw MLException(
			"The selected mesh has no attribute by name \""
			ATTRIBUTE_MESH_TO_SKELETON_INDEX_NAME "\"."
		);
	}

	return vertices;
}

static bool isContainedInCone(
	Point const& origin, Point const& point,
	Scalarm cone_angle, Normal cone_direction);

Point calculateBranchExtension(
	Point const& leaf_vertex,
	std::vector<CVertexO const*> const& leaf_vertices,
	Normal const& leaf_normal,
	float         angle)
{
	Point  total = { 0, 0, 0 };
	size_t count = 0;
	for (CVertexO const* vertex : leaf_vertices)
	{
		if ( isContainedInCone(leaf_vertex, vertex->P(), angle, leaf_normal) )
		{
			count++;
			total += vertex->P();
		}
	}

	if (count > 0)
		total /= count;
	return total;
}

bool isContainedInCone(
	Point const& origin, Point const& point,
	Scalarm cone_angle, Normal cone_direction)
{
	Normal  vert_normal  = (point - origin).Normalize();
	Scalarm normal_angle = vcg::AngleN(cone_direction, vert_normal);

	return normal_angle <= cone_angle;
}

}
