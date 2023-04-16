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

#include "BranchMeasureFilter.h"

#include "common/AlgorithmSkeletonize.h"
#include "common/BranchExtender.h"
#include "common/SkeletonMesh.h"

#include <vcg/complex/algorithms/isotropic_remeshing.h>

#define ATTRIBUTE_PARENT_HOLE_FACES "parent_hole"

namespace curvatureSkeleton
{

static CMeshO duplicateAndExtendMeshBase(CMeshO& mesh);
static void cutSkeletonToOriginalMesh(CMeshO& mesh, SkeletonMesh& skeleton);
static Scalarm computeLongestPath(SkeletonMesh& mesh, SkeletonVertex* base);

std::map<std::string, QVariant> BranchMeasureFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos* cb)
{
	//get parameters
	auto only_selected = params.getBool(PARAM_MEASURE_ONLY_SELECTED);
	auto save_skeletons = params.getBool(PARAM_SAVE_SKELETONS);

	std::vector<MeshModel*> models;
	if (only_selected)
	{
		models.emplace_back(document.mm());
	}
	else
	{
		for (auto& mesh_mm : document.meshIterator())
		{
			if ( mesh_mm.isVisible() )
				models.emplace_back(&mesh_mm);
		}
	}

	//foreach mesh
	for (int i = 0; i < models.size(); i++)
	{
		auto* mesh_mm = models[i];
		auto mesh_label = mesh_mm->label();
		auto& mesh = mesh_mm->cm;

		cb((i + 1) * 100.0 / models.size(), QString("Measuring %1...").arg(mesh_label).toStdString().c_str());
		auto clone_mesh = duplicateAndExtendMeshBase(mesh);

		//compute skeleton
		auto params = AlgorithmSkeletonize::Parameters();
		params.max_iterations = 200;
		params.skeletonizer_params = AlgorithmSkeletonize::Skeletonizer::Parameters(clone_mesh, 0.002);
		params.skeletonizer_params.medially_centering_speed_tradeoff = 50;
		params.skeletonizer_params.quality_speed_tradeoff = 40;
		params.save_mesoskeletons = false;

		auto skeleton = AlgorithmSkeletonize(vcg::DummyCallBackPos, plugin).skeletonize(clone_mesh, params, false);

		//extend skeleton
		Scalarm cone_extension_angle = 25.f;
		int search_depth = 20;
		BranchExtender::extendLeafs(clone_mesh, skeleton, vcg::math::ToRad(cone_extension_angle), search_depth);

		//convert skeleton to SkeletonMesh
		SkeletonMesh c_skeleton;
		vcg::tri::Append<SkeletonMesh, CMeshO>::MeshCopyConst(c_skeleton, skeleton);
		vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(c_skeleton);
		vcg::tri::InitVertexIMark(c_skeleton);

		cutSkeletonToOriginalMesh(mesh, c_skeleton);
		vcg::tri::Allocator<SkeletonMesh>::CompactEveryVector(c_skeleton);
		vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(c_skeleton);

		Scalarm longest_path = 0.0;
		for (auto& vert : c_skeleton.vert)
		{
			if ( !vert.IsD() && vcg::edge::VEDegree<SkeletonEdge>(&vert) == 1 )
				longest_path = std::max(longest_path, computeLongestPath(c_skeleton, &vert));
		}

		plugin.log(
			QString("%2 - Curved Lenght: %1")
			.arg(longest_path, 0, 'f', 3)
			.arg(mesh_label)
			.toStdString()
		);

		//save skeleton
		if (save_skeletons)
		{
			auto* save_skeleton_mm = document.addNewMesh(QString(), QString("%1 - Skeleton").arg(mesh_label), false);
			vcg::tri::Append<CMeshO, SkeletonMesh>::MeshCopyConst(save_skeleton_mm->cm, c_skeleton);
			save_skeleton_mm->updateBoxAndNormals();
		}
	}

	return {};
}

CMeshO duplicateAndExtendMeshBase(CMeshO& mesh)
{
	//duplicate mesh
	CMeshO new_mesh;
	auto parent_hole = vcg::tri::Allocator<CMeshO>::AddPerFaceAttribute<Scalarm>(new_mesh, ATTRIBUTE_PARENT_HOLE_FACES);
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(new_mesh, mesh);

	//find the normal and the barycenter of the interface
	int count = 0; vcg::Point3<Scalarm> normal(0,0,0), barycenter(0,0,0);
	for (auto& face : new_mesh.face)
	{
		if (parent_hole[face] == 1)
		{
			count++;
			normal += face.N();
			barycenter += (face.cP(0) + face.cP(1) + face.cP(2)) / 3.0;
			face.V(0)->SetS();
			face.V(1)->SetS();
			face.V(2)->SetS();
		}
	}

	if (count > 0)
	{
		//find the radius of the interface
		Scalarm max_radius = 0;
		normal.normalize();
		barycenter /= count;
		for (auto& face : new_mesh.face)
		{
			if (parent_hole[face] == 1)
			{
				face.SetS();
				auto radius0 = vcg::SquaredDistance(barycenter, face.cP(0));
				auto radius1 = vcg::SquaredDistance(barycenter, face.cP(1));
				auto radius2 = vcg::SquaredDistance(barycenter, face.cP(2));
				max_radius = std::max(radius0, max_radius);
				max_radius = std::max(radius1, max_radius);
				max_radius = std::max(radius2, max_radius);
			}
		}

		max_radius = vcg::math::Sqrt(max_radius);

		//compute average edge lenght
		Scalarm avg_length = 0;
		for (auto& face : new_mesh.face)
		{
			if (face.IsD()) continue;
			avg_length +=
				vcg::Distance(face.V(0)->cP(), face.V(1)->cP()) +
				vcg::Distance(face.V(1)->cP(), face.V(2)->cP()) +
				vcg::Distance(face.V(2)->cP(), face.V(0)->cP());
		}
		avg_length /= (new_mesh.FN() * 3);

		//move the interface towards the normal direction
		vcg::Point3<Scalarm> movement = normal * max_radius * 2.0;
		for (auto& vertex : new_mesh.vert)
		{
			if (vertex.IsS()) {
				vertex.P() += movement;
				vertex.ClearS();
			}
		}

		//remesh
		new_mesh.face.EnableFFAdjacency();
		new_mesh.vert.EnableVFAdjacency();
		new_mesh.face.EnableVFAdjacency();
		new_mesh.face.EnableMark();
		new_mesh.vert.EnableMark();
		vcg::tri::UpdateTopology<CMeshO>::FaceFace(new_mesh);
		vcg::tri::UpdateTopology<CMeshO>::VertexFace(new_mesh);
		vcg::tri::UpdateSelection<CMeshO>::FaceDilate(new_mesh);

		vcg::tri::IsotropicRemeshing<CMeshO>::Params params;
		params.SetFeatureAngleDeg(181.0f);
		params.adapt = false;
		params.selectedOnly = true;
		params.splitFlag = true;
		params.collapseFlag = true;
		params.swapFlag = true;
		params.smoothFlag = true;
		params.projectFlag = false;
		params.surfDistCheck = false;
		params.SetTargetLen(avg_length);
		params.iter = 5;

		vcg::tri::InitFaceIMark(new_mesh);
		vcg::tri::InitVertexIMark(new_mesh);
		vcg::tri::IsotropicRemeshing<CMeshO>::Do(new_mesh, params);
		vcg::tri::UpdateSelection<CMeshO>::Clear(new_mesh);
	}

	return new_mesh;
}

void cutSkeletonToOriginalMesh(CMeshO& mesh, SkeletonMesh& skeleton)
{
	//find the interface of the mesh
	auto parent_hole = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(mesh, ATTRIBUTE_PARENT_HOLE_FACES);
	std::vector<CFaceO const*> hole_interface;
	for (auto& face : mesh.face)
	{
		if (parent_hole[face] == 1)
			hole_interface.push_back(&face);
	}

	if (hole_interface.empty())
		return;

	std::vector<SkeletonEdge*> edges_to_remove;
	std::unordered_set<SkeletonVertex*> vertices_to_remove;
	std::unordered_set<SkeletonVertex*> vertices_on_the_border;

	//find edges and vertices that are outside of the mesh's interface
	for (auto& vertex : skeleton.vert) {
		vcg::Point3<Scalarm> min_closest_point(vertex.cP()), closest_normal(0, 0, 0);
		Scalarm min_distance = std::numeric_limits<Scalarm>::max();
		for (auto* face : hole_interface)
		{
			Scalarm distance = std::numeric_limits<Scalarm>::max();
			vcg::Point3<Scalarm> closest_point(vertex.cP());
			vcg::face::PointDistanceBase(*face, vertex.cP(), distance, closest_point);

			if (distance < min_distance) {
				closest_normal = face->N();
				min_distance = distance;
				min_closest_point = closest_point;
			}
		}

		bool is_vertex_outside = (vertex.cP() - min_closest_point).dot(closest_normal) > 0;
		if (is_vertex_outside) {
			vertices_to_remove.insert(&vertex);
			vertex.P() = min_closest_point;
		}
	}

	for (auto& edge : skeleton.edge) {
		bool is_vertex0_outside = vertices_to_remove.count(edge.V(0));
		bool is_vertex1_outside = vertices_to_remove.count(edge.V(1));

		if (is_vertex0_outside && is_vertex1_outside) {
			edges_to_remove.push_back(&edge);
		}
		else if (is_vertex0_outside) {
			vertices_on_the_border.insert(edge.V(0));
		}
		else if (is_vertex1_outside) {
			vertices_on_the_border.insert(edge.V(1));
		}
	}

	//remove such edges and vertices
	for (auto* edge : edges_to_remove)
		vcg::tri::Allocator<SkeletonMesh>::DeleteEdge(skeleton, *edge);

	for (auto* vertex : vertices_to_remove)
	{
		if(vertices_on_the_border.count(vertex) == 0)
			vcg::tri::Allocator<SkeletonMesh>::DeleteVertex(skeleton, *vertex);
	}
}

Scalarm computeLongestPath(SkeletonMesh& mesh, SkeletonVertex* base)
{
	std::queue< SkeletonVertex* >frontier;
	auto longest_path = vcg::tri::Allocator<SkeletonMesh>::AddPerVertexAttribute<Scalarm>(mesh);
	for (auto& vert : mesh.vert)
		longest_path[vert] = 0;

	Scalarm longest_path_lenght = 0.0;

	vcg::tri::UnMarkAll(mesh);

	frontier.emplace(base);
	std::vector<SkeletonVertex*> star;
	while (!frontier.empty())
	{
		auto* current = frontier.front();
		frontier.pop();

		vcg::tri::Mark(mesh, current);
		vcg::edge::VVStarVE(current, star);
		for (auto* adj : star)
		{
			if( vcg::tri::IsMarked(mesh, adj) )
				continue;

			frontier.push(adj);

			auto distance = vcg::Distance(current->cP(), adj->cP());
			longest_path[adj] = longest_path[current] + distance;
			longest_path_lenght = std::max(longest_path_lenght, longest_path[adj]);
		}
	}

	vcg::tri::Allocator<SkeletonMesh>::DeletePerVertexAttribute(mesh, longest_path);
	return longest_path_lenght;
}

}
