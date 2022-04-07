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

#include "common/additionalAttributeNames.h"

#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <functional>
#include <common/plugins/interfaces/filter_plugin.h>

namespace curvatureSkeleton
{

typedef vcg::Point3<Scalarm>             Point;
typedef vcg::Point3<Scalarm>             Normal;
typedef std::reference_wrapper<CVertexO const> CVertexORef;
typedef vcg::tri::Allocator<CMeshO>      Allocator;

struct SkeletonLeaf
{
	uint      index;
	CVertexO* vertex;
	Normal    normal;
};

static std::vector<SkeletonLeaf> findSkeletonLeafs(CMeshO const& skeleton);
static void extendBranch(SkeletonLeaf const& leaf, CMeshO const& mesh, CMeshO& skeleton, float angle);

void extendLeafBranches(CMeshO const& mesh, CMeshO& skeleton, float angle)
{
	auto vertices = findSkeletonLeafs(skeleton);
	for (auto& leaf : vertices)
	{
		extendBranch(leaf, mesh, skeleton, angle);
	}
}

static uint findVertexIndex(CMeshO::VertContainer const& vertices, CVertexO const* vertex);

std::vector<SkeletonLeaf> findSkeletonLeafs(CMeshO const& skeleton)
{
	std::unordered_set<CVertexO const*>            found_vertices;
	std::unordered_map<CVertexO*, CVertexO const*> border_vertices;

	for (auto& edge : skeleton.edge)
	{
		auto v0 = edge.cV(0);
		auto v1 = edge.cV(1);

		if (found_vertices.insert(v0).second)
			border_vertices.insert({v0, v1});
		else
			border_vertices.erase(v0);

		if (found_vertices.insert(v1).second)
			border_vertices.insert({v1, v0});
		else
			border_vertices.erase(v1);
	}

	std::vector<SkeletonLeaf> vertices;
	vertices.reserve( border_vertices.size() );
	for (auto& edge_verts : border_vertices)
	{
		SkeletonLeaf leaf;
		leaf.index  = findVertexIndex(skeleton.vert, edge_verts.first);
		leaf.vertex = edge_verts.first;
		leaf.normal = (edge_verts.first->cP() - edge_verts.second->cP()).Normalize();
		vertices.push_back(leaf);
	}

	return vertices;
}

uint findVertexIndex(CMeshO::VertContainer const& vertices, CVertexO const* vertex)
{
	for (uint i = 0; i < vertices.size(); i++)
	{
		if (&vertices[i] == vertex)
			return i;
	}
	throw MLException("Given vertex is not contained in the given vector.");
}

static std::vector<CVertexORef> getMeshLeafVertices(uint vertex_index, CMeshO const& mesh);
static Point					calculateBranchExtension(
									Point const& left_vertex,
									std::vector<CVertexORef> const& mesh_leaf_vertices,
									Normal const& leaf_normal,
									float angle);

void extendBranch(SkeletonLeaf const& leaf, CMeshO const& mesh, CMeshO& skeleton, float angle)
{
	auto vertices = getMeshLeafVertices(leaf.index, mesh);
	auto new_point = calculateBranchExtension(leaf.vertex->cP(), vertices, leaf.normal, angle);

	Allocator::AddVertex(skeleton, new_point);
	Allocator::AddEdge(skeleton, leaf.index, (skeleton.VN() - 1));
}

std::vector<CVertexORef> getMeshLeafVertices(uint vertex_index, CMeshO const& mesh)
{
	std::vector<CVertexORef> vertices;
	auto iterator = vcg::tri::Allocator<CMeshO>::FindPerVertexAttribute<uint>(
        mesh, ATTRIBUTE_MESH_TO_SKELETON_INDEX_NAME);

	if (vcg::tri::Allocator<CMeshO>::IsValidHandle(mesh, iterator))
	{
		for (uint i = 0; i < mesh.vert.size(); i++)
		{
			if (iterator[i] == vertex_index)
			{
				vertices.push_back(mesh.vert[i]);
			}
		}
	}
	else {
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
	Point const&                    leaf_vertex,
	std::vector<CVertexORef> const& mesh_leaf_vertices,
	Normal const&                   leaf_normal,
	float                           angle)
{
	Point  total = { 0, 0, 0 };
	size_t count = 0;
	for (CVertexO const& vertex : mesh_leaf_vertices)
	{
		if (isContainedInCone(leaf_vertex, vertex.P(), angle, leaf_normal))
		{
			count++;
			total += vertex.P();
		}
	}
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
