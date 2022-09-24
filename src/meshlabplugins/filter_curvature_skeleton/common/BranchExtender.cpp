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

#include "SkeletonMesh.h"

#include <vector>
#include <unordered_set>
#include <common/plugins/interfaces/filter_plugin.h>
#include <vcg/complex/complex.h>
#include <vcg/complex/append.h>

#define ATTRIBUTE_MESH_TO_SKELETON_INDEX_NAME "skeleton_index"

namespace curvatureSkeleton
{

typedef vcg::Point3<Scalarm> Point;
typedef vcg::Point3<Scalarm> Normal;

typedef vcg::tri::UpdateTopology<SkeletonMesh> SkeletonMeshTopology;
typedef vcg::tri::Allocator<CMeshO> CMeshOAllocator;
typedef vcg::tri::Append<CMeshO, SkeletonMesh> SkeletonToCMeshOAppend;
typedef vcg::tri::Append<SkeletonMesh, CMeshO> CMeshOToSkeletonAppend;

namespace detail
{
	struct SkeletonLeaf
	{
		typedef std::unordered_set<int> BranchIndices;

		int           leaf_index;
		BranchIndices branch_indices;
		Normal        normal;
	};

	static std::vector<SkeletonLeaf> findSkeletonLeafs(SkeletonMesh const& skeleton);
	static void extendBranch(SkeletonLeaf const& leaf, CMeshO const& mesh, CMeshO& skeleton, float angle);

	inline static bool isLeafVertex(SkeletonVertex const& vertex);
	inline static SkeletonLeaf getSkeletonLeafData(SkeletonVertex const& vertex);
	static std::vector<SkeletonVertex const*> getNthParents(SkeletonVertex const& vertex, int max_depth = 5);
	static void getNthParentsRecursive(std::vector<SkeletonVertex const*>& branch, SkeletonVertex const* vertex, SkeletonVertex const* parent, int depth);
	inline static Normal getVertexNormal(SkeletonVertex const& vertex, std::vector<SkeletonVertex const*> const& parents);
	inline static SkeletonLeaf::BranchIndices getBranchIndices(std::vector<SkeletonVertex const*> const& parents);

	static std::vector<CVertexO const*> getMeshLeafVertices(SkeletonLeaf::BranchIndices vertices, CMeshO const& mesh);
	static Point calculateBranchExtension(
		Point const& leaf_vertex,
		std::vector<CVertexO const*> const& leaf_vertices,
		Normal const& leaf_normal,
		float         angle,
		bool& valid_new_point);
	inline static bool isContainedInCone(
		Point const& origin, Point const& point,
		Scalarm cone_angle, Normal cone_direction);
}

void BranchExtender::extendBranch(CMeshO const& mesh, CMeshO& skeleton, int vertex_index, float angle)
{
	SkeletonMesh converted_skeleton;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_skeleton, skeleton);
	SkeletonMeshTopology::VertexEdge(converted_skeleton);

	auto& vertex = converted_skeleton.vert[vertex_index];
	if (detail::isLeafVertex(vertex))
	{
		detail::extendBranch(
			detail::getSkeletonLeafData(vertex),
			mesh, skeleton, angle
		);
	}
}

void BranchExtender::extendLeafs(CMeshO const& mesh, CMeshO& skeleton, float angle)
{
	SkeletonMesh converted_skeleton;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_skeleton, skeleton);
	SkeletonMeshTopology::VertexEdge(converted_skeleton);
	auto vertices = detail::findSkeletonLeafs(converted_skeleton);

	for (auto& leaf : vertices)
	{
		detail::extendBranch(leaf, mesh, skeleton, angle);
	}
}

std::vector<detail::SkeletonLeaf> detail::findSkeletonLeafs(SkeletonMesh const& skeleton)
{
	std::vector<detail::SkeletonLeaf> leafs;
	for (auto& vertex : skeleton.vert)
	{
		if (detail::isLeafVertex(vertex))
			leafs.push_back(
				detail::getSkeletonLeafData(vertex)
			);
	}
	return leafs;
}

inline static bool detail::isLeafVertex(SkeletonVertex const& vertex)
{
	return vcg::edge::VEDegree<SkeletonEdge>(&vertex) == 1;
}

inline static detail::SkeletonLeaf detail::getSkeletonLeafData(SkeletonVertex const& vertex)
{
	auto parents = getNthParents(vertex);

	return {
		vertex.Index(),
		detail::getBranchIndices(parents),
		detail::getVertexNormal(vertex, parents)
	};
}


inline static Normal detail::getVertexNormal(SkeletonVertex const& vertex, std::vector<SkeletonVertex const*> const& parents)
{
	auto& parent = *parents.back();
	if (&parent == &vertex)
	{
		throw std::runtime_error("Normal could not be found");
	}
	else
	{
		auto normal = vertex.cP() - parent.cP();
		normal.normalize();
		return normal;
	}
}

inline static detail::SkeletonLeaf::BranchIndices detail::getBranchIndices(std::vector<SkeletonVertex const*> const& parents)
{
	detail::SkeletonLeaf::BranchIndices indices;
	for (auto& vertex : parents)
	{
		indices.insert(vertex->Index());
	}
	return indices;
}

static std::vector<SkeletonVertex const*> detail::getNthParents(SkeletonVertex const& vertex, int max_depth)
{
	std::vector<SkeletonVertex const*> parents;
	getNthParentsRecursive(parents, &vertex, &vertex, max_depth);
	return parents;
}

static void detail::getNthParentsRecursive(std::vector<SkeletonVertex const*>& branch, SkeletonVertex const* vertex, SkeletonVertex const* parent, int depth)
{
	if (depth == 0)
		return;

	branch.push_back(vertex);

	std::vector<SkeletonVertex*> vertices;
	vcg::edge::VVStarVE(vertex, vertices);
	if (vertices.size() == 1)
	{
		if (vertices[0] != parent)
		{
			detail::getNthParentsRecursive(branch, vertices[0], vertex, depth - 1);
		}
	}
	else if (vertices.size() == 2)
	{
		auto next_vertex = (vertices[0] != parent) ? vertices[0] : vertices[1];
		detail::getNthParentsRecursive(branch, next_vertex, vertex, depth - 1);
	}
}

void detail::extendBranch(SkeletonLeaf const& leaf, CMeshO const& mesh, CMeshO& skeleton, float angle)
{
	auto leaf_vertices = getMeshLeafVertices(leaf.branch_indices, mesh);
	bool valid_new_point = false;
	auto new_point = calculateBranchExtension(skeleton.vert[leaf.leaf_index].cP(), leaf_vertices, leaf.normal, angle, valid_new_point);
	if (valid_new_point)
	{
		CMeshOAllocator::AddVertex(skeleton, new_point);
		CMeshOAllocator::AddEdge(skeleton, leaf.leaf_index, (skeleton.VN() - 1));
	}
}

std::vector<CVertexO const*> detail::getMeshLeafVertices(detail::SkeletonLeaf::BranchIndices branch_vertices, CMeshO const& mesh)
{
	std::vector<CVertexO const*> vertices;
	auto iterator = vcg::tri::Allocator<CMeshO>::FindPerVertexAttribute<Scalarm>(
        mesh, ATTRIBUTE_MESH_TO_SKELETON_INDEX_NAME);

	if (vcg::tri::Allocator<CMeshO>::IsValidHandle(mesh, iterator))
	{
		for (auto& vertex : mesh.vert)
		{
			if (branch_vertices.count(static_cast<int>(iterator[vertex])) > 0)
			{
				vertices.push_back( &vertex );
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

Point detail::calculateBranchExtension(
	Point const& leaf_vertex,
	std::vector<CVertexO const*> const& leaf_vertices,
	Normal const& leaf_normal,
	float         angle,
	bool&		  valid_new_point)
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
	{
		valid_new_point = true;
		return total / count;
	}
	else
	{
		valid_new_point = false;
		return Point();
	}
}

bool detail::isContainedInCone(
	Point const& origin, Point const& point,
	Scalarm cone_angle, Normal cone_direction)
{
	Scalarm normal_angle = vcg::Angle(cone_direction, point - origin);
	return normal_angle <= cone_angle;
}

}
