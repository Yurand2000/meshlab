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
#include <limits>
#include <common/plugins/interfaces/filter_plugin.h>
#include <vcg/complex/complex.h>
#include <vcg/complex/append.h>
#include <vcg/complex/algorithms/intersection.h>
#include <vcg/complex/algorithms/geodesic.h>

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
		typedef std::deque<int> BranchIndices;

		int           leaf_index;
		BranchIndices branch_indices;
		Normal        normal;
	};

	static std::vector<SkeletonLeaf> findSkeletonLeafs(SkeletonMesh const& skeleton, int max_depth = 5);
	static void extendBranch(SkeletonLeaf leaf, CMeshO const& mesh, CMeshO& skeleton, float angle);

	inline static bool isLeafVertex(SkeletonVertex const& vertex);
	inline static SkeletonLeaf getSkeletonLeafData(SkeletonVertex const& vertex, int max_depth = 5);
	static std::vector<SkeletonVertex const*> getNthParents(SkeletonVertex const& vertex, int max_depth = 5);
	static void getNthParentsRecursive(std::vector<SkeletonVertex const*>& branch, SkeletonVertex const* vertex, SkeletonVertex const* parent, int depth);
	inline static Normal getVertexNormal(SkeletonVertex const& vertex, std::vector<SkeletonVertex const*> const& parents);
	inline static SkeletonLeaf::BranchIndices getBranchIndices(std::vector<SkeletonVertex const*> const& parents);

	static std::vector<CVertexO const*> getMeshLeafVertices(SkeletonLeaf::BranchIndices vertices, CMeshO const& mesh);
	static bool computeBranchExtensionMean(
		Point const& leaf_vertex,
		std::vector<CVertexO const*> const& leaf_vertices,
		Normal const& leaf_normal,
		float         angle,
		Point& new_point);
	static bool computeBranchExtensionGeodesic(
		CMeshO const& mesh,
		Point const& leaf_vertex,
		std::vector<CVertexO const*> const& leaf_vertices,
		Normal const& leaf_normal,
		float         angle,
		Point& new_point);
	static bool computeBranchExtensionMeanSurface(
		Point const& leaf_vertex,
		std::vector<CFaceO> const& leaf_faces,
		Normal const& leaf_normal,
		float         angle,
		Point& new_point);
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

void BranchExtender::extendLeafs(CMeshO const& mesh, CMeshO& skeleton, float angle, int normal_search_depth)
{
	SkeletonMesh converted_skeleton;
	CMeshOToSkeletonAppend::MeshCopyConst(converted_skeleton, skeleton);
	SkeletonMeshTopology::VertexEdge(converted_skeleton);
	auto vertices = detail::findSkeletonLeafs(converted_skeleton, normal_search_depth);

	for (auto& leaf : vertices)
	{
		detail::extendBranch(leaf, mesh, skeleton, angle);
	}
}

std::vector<detail::SkeletonLeaf> detail::findSkeletonLeafs(SkeletonMesh const& skeleton, int max_depth)
{
	std::vector<detail::SkeletonLeaf> leafs;
	for (auto& vertex : skeleton.vert)
	{
		if (detail::isLeafVertex(vertex))
			leafs.push_back(
				detail::getSkeletonLeafData(vertex, max_depth)
			);
	}
	return leafs;
}

inline static bool detail::isLeafVertex(SkeletonVertex const& vertex)
{
	return vcg::edge::VEDegree<SkeletonEdge>(&vertex) == 1;
}

inline static detail::SkeletonLeaf detail::getSkeletonLeafData(SkeletonVertex const& vertex, int max_depth)
{
	auto parents = getNthParents(vertex, max_depth);

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
		throw MLException("Normal could not be found");
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
		indices.push_back(vertex->Index());
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

void detail::extendBranch(SkeletonLeaf leaf, CMeshO const& mesh, CMeshO& skeleton, float angle)
{
	auto leaf_vertices = getMeshLeafVertices(leaf.branch_indices, mesh);
	bool valid_new_point = false;
	
	Point start_point = skeleton.vert[leaf.leaf_index].P();
	Point new_point; bool extendingSuccessful; Scalarm delta; size_t iter = 0;
	const Scalarm min_delta = mesh.bbox.SquaredDiag() * Scalarm(0.01 * 0.01);
	const size_t max_iter = 10;

	//iteratively approach the mesh
	do {
		extendingSuccessful = computeBranchExtensionGeodesic(mesh, skeleton.vert[leaf.leaf_index].cP(), leaf_vertices, leaf.normal, angle, new_point);
		if (extendingSuccessful) {
			// compute the new point to add
			auto old_leaf = skeleton.vert[leaf.leaf_index].cP();
			delta = (new_point - old_leaf).Norm();
			Point normal = (new_point - old_leaf).normalized();
			new_point = old_leaf + (normal * 0.5 * delta / double(max_iter - iter));

			// add the new vertex (and edge)
			vcg::tri::Allocator<CMeshO>::AddVertex(skeleton, new_point);
			vcg::tri::Allocator<CMeshO>::AddEdge(skeleton, &skeleton.vert[leaf.leaf_index], &skeleton.vert.back());

			// update leaf data
			leaf.branch_indices.pop_back();
			leaf.branch_indices.push_front(skeleton.vert.back().Index());
			leaf.leaf_index = skeleton.vert.back().Index();

			// compute the updated normal
			leaf.normal = (new_point - skeleton.vert[leaf.branch_indices.back()].cP()).normalized();
		}
		iter++;
	}
	while (extendingSuccessful && delta > min_delta && iter < max_iter);

	//try to raycast the new point onto the surface
	if (extendingSuccessful) {
		auto new_normal = (new_point - start_point).normalized();

		Point raycast_point;
		if ( vcg::IntersectionRayMesh(mesh, vcg::Ray3<Scalarm>(start_point, new_normal), raycast_point) ) {
			skeleton.vert[leaf.leaf_index].P() = raycast_point;
		}
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
			if (std::count(branch_vertices.begin(), branch_vertices.end(), static_cast<int>(iterator[vertex])) > 0) {
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

bool detail::computeBranchExtensionMean(
	Point const& leaf_vertex,
	std::vector<CVertexO const*> const& leaf_vertices,
	Normal const& leaf_normal,
	float         angle,
	Point&		  new_point)
{
	Point  total = { 0, 0, 0 };
	size_t count = 0;
	for (CVertexO const* vertex : leaf_vertices)
	{
		if ( isContainedInCone(leaf_vertex, vertex->cP(), angle, leaf_normal) )
		{
			count++;
			total += vertex->cP();
		}
	}

	if (count > 0)
	{
		new_point = total / count;
		return true;
	}
	else
	{
		return false;
	}
}

bool detail::computeBranchExtensionGeodesic(
	CMeshO const& mesh,
	Point const& leaf_vertex,
	std::vector<CVertexO const*> const& leaf_vertices,
	Normal const& leaf_normal,
	float         angle,
	Point& new_point)
{
	// generate N rays from the center, on the cone, distributed evenly on a circle
	const size_t N = 32;
	std::vector<Point> cone_rays(N, Point());

		// generate a single ray, rotated at "angle" from the Y axis, in any direction
	cone_rays[0] = vcg::RotationMatrix(Point(1, 0, 0), double(angle)) * Point(0, 1, 0);

		// copy the ray N-1 times, rotating by (2pi/angle) around the Y axis.
	for (size_t i = 1; i < N; i++) {
		cone_rays[i] = vcg::RotationMatrix(Point(0, 1, 0), (double(i)*(2*M_PI)/N)) * cone_rays[0];
	}

		// rotate all the rays to the leaf normal (shortest arc is fine)
	auto rotation_axis = (Point(0, 1, 0) ^ leaf_normal).normalized();
	auto rotation_angle = vcg::Angle(Point(0, 1, 0), leaf_normal);

	auto rotation_matrix = vcg::RotationMatrix(rotation_axis, rotation_angle);
	for (size_t i = 0; i < N; i++) {
		cone_rays[i] = (rotation_matrix * cone_rays[i]).normalized();
	}

		// project the rays on the original mesh
	std::vector<Point> raycast_points;
	raycast_points.reserve(N);

	for (size_t i = 0; i < N; i++) {
		Point raycast_point;
		if (vcg::IntersectionRayMesh(mesh, vcg::Ray3<Scalarm>(leaf_vertex, cone_rays[i]), raycast_point)) {
			raycast_points.push_back(raycast_point);
		}
	}

	if (raycast_points.empty()) {
		return false;
	}

	// duplicate the points inside the cone in a separate mesh
	CMeshO copy, cone_only;
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopyConst(copy, mesh, false);
	vcg::tri::UpdateSelection<CMeshO>::Clear(copy);

	for (CVertexO& vertex : copy.vert) {
		if (isContainedInCone(leaf_vertex, vertex.cP(), angle, leaf_normal)) {
			vertex.SetS();
		}
	}

	vcg::tri::UpdateSelection<CMeshO>::FaceFromVertexLoose(copy);
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(cone_only, copy, true);

	if (cone_only.vert.empty()) {
		return false;
	}

	// find the N closest points in the cone, use those as basis for the geodesic
	std::vector<CVertexO*> closest_points;
	closest_points.reserve(raycast_points.size());

	for (Point& raycast_point : raycast_points) {
		CVertexO* best_point = nullptr;
		auto best_distance = std::numeric_limits<double>::max();

		for (CVertexO& vertex : cone_only.vert)	{
			auto distance = vcg::SquaredDistance(raycast_point, vertex.cP());

			if (distance < best_distance) {
				best_distance = distance;
				best_point = &vertex;
			}
		}

		bool skip_push = false;
		for (CVertexO* closest_point : closest_points) {
			if (closest_point == best_point) {
				skip_push = true;
				break;
			}
		}

		if (!skip_push)
			closest_points.push_back(best_point);
	}

	if (closest_points.empty()) {
		throw MLException("Failed to select closest points");
		return false;
	}

	// compute VF adjacency
	cone_only.face.EnableVFAdjacency();
	cone_only.vert.EnableVFAdjacency();
	vcg::tri::RequireVFAdjacency(cone_only);
	vcg::tri::RequirePerVertexQuality(cone_only);
	vcg::tri::UpdateTopology<CMeshO>::VertexFace(cone_only);

	// find geodesic furthest
	vcg::tri::Geodesic<CMeshO>::Compute(cone_only, closest_points);
	auto furthest = Point(0, 0, 0);
	Scalarm furthest_distance = 0;
	for (CVertexO const& vert : cone_only.vert) {
		if (vert.cQ() > furthest_distance) {
			furthest_distance = vert.cQ();
			furthest = vert.cP();
		}
	}

	new_point = furthest;
	return true;
}

bool detail::computeBranchExtensionMeanSurface(
	Point const& leaf_vertex,
	std::vector<CFaceO> const& leaf_faces,
	Normal const& leaf_normal,
	float         angle,
	Point& new_point)
{
	Point  total = { 0, 0, 0 };
	Scalarm total_area = 0;
	for (CFaceO const& face : leaf_faces)
	{
		if (
			isContainedInCone(leaf_vertex, face.cP(0), angle, leaf_normal) &&
			isContainedInCone(leaf_vertex, face.cP(1), angle, leaf_normal) &&
			isContainedInCone(leaf_vertex, face.cP(2), angle, leaf_normal)
		) {
			auto center = (face.cP(0) + face.cP(1) + face.cP(2)) / 3.0;
			auto area = vcg::DoubleArea(face) / 2.0;

			total += center * area;
			total_area += area;
		}
	}

	if (total_area > 0)
	{
		new_point = total / total_area;
		return true;
	}
	else
	{
		return false;
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
