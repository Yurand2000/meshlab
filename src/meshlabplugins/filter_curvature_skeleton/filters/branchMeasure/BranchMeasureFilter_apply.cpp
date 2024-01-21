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
#include "common/PruneSkeleton.h"

#include <vcg/complex/algorithms/isotropic_remeshing.h>

#define SEPARATOR ','
#define ATTRIBUTE_PARENT_HOLE_FACES "parent_hole"

namespace curvatureSkeleton
{

static bool tryComputeSkeleton(FilterPlugin const& plugin, QString const& mesh_label, CMeshO& extended_base_mesh, CMeshO& out_skeleton);
static void convertToSkeletonMesh(CMeshO& mesh, CMeshO const& skeleton, Scalarm& longest_curved_path, Scalarm& longest_linear_path, Scalarm& longest_border_lenght, SkeletonMesh& out_skeleton);
static void findTopVertex(SkeletonMesh& skeleton, SkeletonVertex* bottom_vertex, SkeletonVertex*& top_vertex, Scalarm& longest_curved_path);

static void visitSkeleton(SkeletonMesh& mesh, SkeletonVertex& root, std::function<bool(SkeletonVertex const&, SkeletonVertex const&)> traversal_function);

static CMeshO duplicateAndExtendMeshBase(CMeshO& mesh);
static void cutSkeletonToOriginalMesh(CMeshO const& mesh, SkeletonMesh& skeleton, SkeletonVertex*& out_bottom_vertex);
static SkeletonVertex* findLowestPointOnYAxis(SkeletonMesh& skeleton);
static void removeButOneSkeletalConnectedComponents(SkeletonMesh& skeleton);
static Scalarm computeBorderLenght(CMeshO& mesh, SkeletonVertex const& tip);
static Scalarm computeMeshArea(CMeshO& mesh);
static Scalarm computeMeshVolume(CMeshO& mesh);

static const Scalarm cone_extension_angle = 25.f;
static const int cone_extension_search_depth = 20;

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
	auto save_on_file = params.getSaveFileName(PARAM_SAVE_FILE);

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

	QString output_data;
	QTextStream output_stream(&output_data);
	output_stream << "Mesh Name" << SEPARATOR << "Curved Lenght" << SEPARATOR << "Linear Lenght" << SEPARATOR
		<< "Border Lenght" << SEPARATOR << "Surface Area" << SEPARATOR << "Volume" << Qt::endl;

	//foreach mesh
	for (int i = 0; i < models.size(); i++)
	{
		auto* mesh_mm = models[i];
		auto mesh_label = mesh_mm->label();
		auto& mesh = mesh_mm->cm;

		cb((i + 1) * 100.0 / models.size(), QString("Measuring %1...").arg(mesh_label).toStdString().c_str());
		auto clone_mesh = duplicateAndExtendMeshBase(mesh);

		CMeshO skeleton;
		if ( !tryComputeSkeleton(plugin, mesh_label, clone_mesh, skeleton) ) continue;

		//extend skeleton
		BranchExtender::extendLeafs(clone_mesh, skeleton, vcg::math::ToRad(cone_extension_angle), cone_extension_search_depth);

		//convert skeleton to SkeletonMesh
		Scalarm longest_curved_path = 0.0;
		Scalarm longest_linear_path = 0.0;
		Scalarm longest_border_lenght = 0.0;
		SkeletonMesh c_skeleton;
		convertToSkeletonMesh(mesh, skeleton, longest_curved_path, longest_linear_path, longest_border_lenght, c_skeleton);

		//Log Curved Lenght
		plugin.log( QString("%2 - Curved Lenght: %1").arg(longest_curved_path, 0, 'f', 3).arg(mesh_label).toStdString() );

		//Log Linear Lenght
		plugin.log(QString("%2 - Linear Lenght: %1").arg(longest_linear_path, 0, 'f', 3).arg(mesh_label).toStdString());

		//Log Lenght from Border
		if (longest_border_lenght > 0) {
			plugin.log( QString("%2 - Border Lenght: %1").arg(longest_border_lenght, 0, 'f', 3).arg(mesh_label).toStdString() );
		}
		else {
			longest_border_lenght = NAN;
		}

		//Log Surface Area
		auto mesh_area = computeMeshArea(mesh);
		plugin.log( QString("%2 - Surface Area: %1").arg(mesh_area, 0, 'f', 3).arg(mesh_label).toStdString() );

		//Log Volume
		auto mesh_volume = computeMeshVolume(mesh);
		plugin.log( QString("%2 - Volume: %1").arg(mesh_volume, 0, 'f', 3).arg(mesh_label).toStdString() );

		//save skeleton
		if (save_skeletons)
		{
			auto* save_skeleton_mm = document.addNewMesh(QString(), QString("%1 - Skeleton").arg(mesh_label), false);
			vcg::tri::Append<CMeshO, SkeletonMesh>::MeshCopyConst(save_skeleton_mm->cm, c_skeleton);
			save_skeleton_mm->updateBoxAndNormals();
		}

		//save per mesh data on file
		output_stream << mesh_label << SEPARATOR << longest_curved_path << SEPARATOR << longest_linear_path << SEPARATOR
			<< longest_border_lenght << SEPARATOR << mesh_area << SEPARATOR << mesh_volume << Qt::endl;

	}

	if (!save_on_file.isEmpty())
	{
		if (!save_on_file.endsWith(".csv"))
			save_on_file.append(".csv");

		QFile out_file(save_on_file);
		if (!out_file.open(QIODevice::WriteOnly | QIODevice::Text))
			plugin.log(QString("Unable to open file %1").arg(save_on_file).toStdString());
		else {
			QTextStream out_file_stream(&out_file);
			output_stream.flush();
			out_file_stream << output_data;
			out_file.close();
		}
	}

	return {};
}

bool tryComputeSkeleton(
	FilterPlugin const& plugin,
	QString const& mesh_label,
	CMeshO& extended_base_mesh,
	CMeshO& out_skeleton
) {
	auto params = AlgorithmSkeletonize::Parameters();
	params.max_iterations = 200;
	params.skeletonizer_params = AlgorithmSkeletonize::Skeletonizer::Parameters(extended_base_mesh, 0.002);
	params.skeletonizer_params.medially_centering_speed_tradeoff = 50;
	params.skeletonizer_params.quality_speed_tradeoff = 40;
	params.save_mesoskeletons = false;

	try {
		out_skeleton = AlgorithmSkeletonize(vcg::DummyCallBackPos, plugin).skeletonize(extended_base_mesh, params, false);
		return true;
	}
	catch (MLException e) {
		plugin.log(QString("Error when measuring mesh %1: %2").arg(mesh_label).arg(e.what()).toStdString());
		return false;
	}
}

void convertToSkeletonMesh(
	CMeshO& non_extended_mesh,
	CMeshO const& skeleton,
	Scalarm& longest_curved_path,
	Scalarm& longest_linear_path,
	Scalarm& longest_border_lenght,
	SkeletonMesh& c_skeleton
) {
	vcg::tri::Append<SkeletonMesh, CMeshO>::MeshCopyConst(c_skeleton, skeleton);
	vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(c_skeleton);
	vcg::tri::InitVertexIMark(c_skeleton);

	SkeletonVertex* bottom_vertex = nullptr;
	cutSkeletonToOriginalMesh(non_extended_mesh, c_skeleton, bottom_vertex);
	removeButOneSkeletalConnectedComponents(c_skeleton);
	vcg::tri::UpdateTopology<SkeletonMesh>::VertexEdge(c_skeleton);

	if (bottom_vertex == nullptr)
		return;

	//find top vertex
	SkeletonVertex* top_vertex = nullptr;
	findTopVertex(c_skeleton, bottom_vertex, top_vertex, longest_curved_path);

	if (top_vertex == nullptr)
		return;

	//compute lenghts
	//longest_curved_path is already computed
	longest_linear_path = vcg::Distance(bottom_vertex->cP(), top_vertex->cP());
	longest_border_lenght = computeBorderLenght(non_extended_mesh, *top_vertex);

	//prune all the other branches
	vcg::tri::UpdateSelection<SkeletonMesh>::Clear(c_skeleton);
	for (auto& vert : c_skeleton.vert) {
		if (!vert.IsD() && vcg::edge::VEDegree<SkeletonEdge>(&vert) == 1 && &vert != bottom_vertex && &vert != top_vertex) {
			vert.SetS();
		}
	}
	PruneSkeleton::pruneSkeletonNoRemap(c_skeleton, Scalarm(0), true);

	return;
}

void findTopVertex(
	SkeletonMesh& skeleton,
	SkeletonVertex* bottom_vertex,
	SkeletonVertex*& top_vertex,
	Scalarm& longest_curved_path
) {
	top_vertex = nullptr;
	longest_curved_path = Scalarm(0);
	auto longest_path = vcg::tri::Allocator<SkeletonMesh>::AddPerVertexAttribute<Scalarm>(skeleton);
	for (auto& vert : skeleton.vert) longest_path[vert] = 0;

	visitSkeleton(skeleton, *bottom_vertex, [&](SkeletonVertex const& current, SkeletonVertex const& neighbor) {
		auto distance = vcg::Distance(current.cP(), neighbor.cP());
		longest_path[neighbor] = longest_path[current] + distance;

		if (longest_path[neighbor] > longest_curved_path) {
			top_vertex = &skeleton.vert[neighbor.Index()];
			longest_curved_path = longest_path[neighbor];
		}
		return true;
	});

	vcg::tri::Allocator<SkeletonMesh>::DeletePerVertexAttribute(skeleton, longest_path);
}

CMeshO duplicateAndExtendMeshBase(CMeshO& mesh)
{
	//duplicate mesh
	CMeshO new_mesh;
	auto parent_hole = vcg::tri::Allocator<CMeshO>::AddPerFaceAttribute<Scalarm>(new_mesh, ATTRIBUTE_PARENT_HOLE_FACES);
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(new_mesh, mesh);

	//find the normal and the barycenter of the interface
	//select the vertices for later moving
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
		normal.normalize();
		barycenter /= count;

		//find the radius of the interface
		Scalarm max_radius = 0;
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

	//take only the biggest connected component
	std::vector< std::pair<int, CMeshO::FacePointer> > ccs;
	new_mesh.face.EnableFFAdjacency();
	vcg::tri::UpdateTopology<CMeshO>::FaceFace(new_mesh);
	vcg::tri::Clean<CMeshO>::ConnectedComponents(new_mesh, ccs);

	size_t biggest = 0; int max_size = -1;
	for (size_t i = 0; i < ccs.size(); i++) {
		if (ccs[i].first > max_size) {
			max_size = ccs[i].first;
			biggest = i;
		}
	}

	if (max_size == -1) {
		return new_mesh;
	}
	else {
		vcg::tri::UpdateSelection<CMeshO>::Clear(new_mesh);
		ccs[biggest].second->SetS();
		vcg::tri::UpdateSelection<CMeshO>::FaceConnectedFF(new_mesh);
		vcg::tri::UpdateSelection<CMeshO>::VertexFromFaceLoose(new_mesh);
		CMeshO clean_new_mesh;
		vcg::tri::Append<CMeshO, CMeshO>::MeshCopyConst(clean_new_mesh, new_mesh, true);

		return clean_new_mesh;
	}
}

void cutSkeletonToOriginalMesh(CMeshO const& mesh, SkeletonMesh& skeleton, SkeletonVertex*& out_bottom_vertex)
{
	//find the interface of the mesh
	std::vector<CFaceO const*> hole_interface;
	auto parent_hole = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(mesh, ATTRIBUTE_PARENT_HOLE_FACES);
	if (vcg::tri::Allocator<CMeshO>::IsValidHandle<Scalarm>(mesh, parent_hole)) {
		for (auto& face : mesh.face)
		{
			if (parent_hole[face] == 1)
				hole_interface.push_back(&face);
		}
	}

	if (hole_interface.empty()) {
		out_bottom_vertex = findLowestPointOnYAxis(skeleton);
		return;
	}

	//find edges that intersect the interface
	vcg::tri::UnMarkAll(skeleton);
	std::unordered_set<SkeletonVertex*> vertices_on_the_border;
	vcg::Point3d intersection_point;
	for (auto& edge : skeleton.edge) {
		auto segment = vcg::Segment3<Scalarm>(edge.cP(0), edge.cP(1));

		for (auto* face : hole_interface) {
			auto triangle = vcg::Triangle3<Scalarm>(face->cP(0), face->cP(1), face->cP(2));

			Scalarm barycentric_coords[3];
			if (vcg::IntersectionSegmentTriangle(
				segment, triangle,
				barycentric_coords[1],
				barycentric_coords[2]
			)) {
				barycentric_coords[0] = 1.0 - barycentric_coords[1] - barycentric_coords[2];
				intersection_point =
					face->cP(0) * barycentric_coords[0] +
					face->cP(1) * barycentric_coords[1] +
					face->cP(2) * barycentric_coords[2];

				bool is_vertex_0_outside = (edge.cP(0) - intersection_point).dot(face->cN()) > 0;
				if (is_vertex_0_outside) {
					vertices_on_the_border.insert(edge.V(0));
					edge.P(0) = intersection_point;
					vcg::tri::Mark(skeleton, edge.V(1));
				}
				else {
					vertices_on_the_border.insert(edge.V(1));
					edge.P(1) = intersection_point;
					vcg::tri::Mark(skeleton, edge.V(0));
				}

				break;
			}
		}
	}

	//find vertices that are outside of the mesh's interface
	std::queue<SkeletonVertex*> temp_vertices_queue;
	for (auto* vertex : vertices_on_the_border)
		temp_vertices_queue.push(vertex);

	std::unordered_set<SkeletonVertex*> vertices_to_remove;
	while ( !temp_vertices_queue.empty() )
	{
		auto* curr = temp_vertices_queue.front();
		temp_vertices_queue.pop();

		vcg::tri::Mark(skeleton, curr);
		vertices_to_remove.insert(curr);

		std::vector<SkeletonVertex*> star;
		vcg::edge::VVStarVE(curr, star);
		for (auto* adj : star)
		{
			if (!vcg::tri::IsMarked(skeleton, adj)) {
				temp_vertices_queue.push(adj);
			}
		}
	}

	for (auto* vertex : vertices_on_the_border)
		vertices_to_remove.erase(vertex);

	//find edges that are outside of the mesh's interface
	std::vector<SkeletonEdge*> edges_to_remove;
	for (auto& edge : skeleton.edge) {
		bool is_vertex0_outside = vertices_to_remove.count(edge.V(0)) > 0;
		bool is_vertex1_outside = vertices_to_remove.count(edge.V(1)) > 0;

		if (is_vertex0_outside || is_vertex1_outside) {
			edges_to_remove.push_back(&edge);
		}
	}

	//remove such edges and vertices
	for (auto* edge : edges_to_remove)
		vcg::tri::Allocator<SkeletonMesh>::DeleteEdge(skeleton, *edge);

	for (auto* vertex : vertices_to_remove)
		vcg::tri::Allocator<SkeletonMesh>::DeleteVertex(skeleton, *vertex);

	vcg::tri::Allocator<SkeletonMesh>::CompactEveryVector(skeleton);

	//find the bottom vertex
	out_bottom_vertex = nullptr;
	for (auto& vertex : skeleton.vert) {
		if (vertex.cP() == intersection_point)
			out_bottom_vertex = &vertex;
	}

	if (out_bottom_vertex == nullptr) {
		out_bottom_vertex = findLowestPointOnYAxis(skeleton);
	}
}

SkeletonVertex* findLowestPointOnYAxis(SkeletonMesh& skeleton) {
	SkeletonVertex* bottom_vertex = nullptr;
	Scalarm lowest_y = std::numeric_limits<Scalarm>::max();
	for (auto& vert : skeleton.vert)
	{
		if (vert.cP().Y() < lowest_y)
		{
			lowest_y = vert.cP().Y();
			bottom_vertex = &vert;
		}
	}

	return bottom_vertex;
}

void removeButOneSkeletalConnectedComponents(SkeletonMesh& skeleton)
{
	std::vector<std::pair<int, SkeletonEdge*>> conn_comps;
	vcg::tri::Clean<SkeletonMesh>::edgeMeshConnectedComponents(skeleton, conn_comps);

	std::sort(conn_comps.begin(), conn_comps.end(),
		[](std::pair<int, SkeletonEdge*>& lpair, std::pair<int, SkeletonEdge*>& rpair) { return lpair.first > rpair.first; });

	vcg::tri::EdgeConnectedComponentIterator<SkeletonMesh> iterator;
	for (int i = 1; i < conn_comps.size(); i++)
	{
		for (iterator.start(skeleton, conn_comps[i].second); !iterator.completed(); ++iterator) {
			auto* edge = *iterator;

			if( !edge->IsD() )
			{
				vcg::tri::Allocator<SkeletonMesh>::DeleteEdge(skeleton, *edge);
				if ( !edge->V(0)->IsD() )
					vcg::tri::Allocator<SkeletonMesh>::DeleteVertex(skeleton, *edge->V(0));
				if ( !edge->V(1)->IsD() )
					vcg::tri::Allocator<SkeletonMesh>::DeleteVertex(skeleton, *edge->V(1));
			}
		}
	}

	vcg::tri::Allocator<SkeletonMesh>::CompactEveryVector(skeleton);
}

void visitSkeleton(
	SkeletonMesh& mesh,
	SkeletonVertex& root,
	std::function<bool(SkeletonVertex const&, SkeletonVertex const&)> traversal_function
) {
	std::queue< SkeletonVertex* >frontier;
	vcg::tri::UnMarkAll(mesh);

	frontier.emplace(&root);
	std::vector<SkeletonVertex*> star;
	while (!frontier.empty())
	{
		auto* current = frontier.front();
		frontier.pop();

		vcg::tri::Mark(mesh, current);
		vcg::edge::VVStarVE(current, star);
		for (auto* adj : star)
		{
			if (vcg::tri::IsMarked(mesh, adj))
				continue;

			if (traversal_function(*current, *adj))
				frontier.push(adj);
		}
	}
}

Scalarm computeBorderLenght(CMeshO& mesh, SkeletonVertex const& tip)
{
	//from the interface border of the mesh compute the average distance from the skeleton tip
	Scalarm distance = 0;
	int count = 0;

	vcg::tri::UpdateSelection<CMeshO>::FaceClear(mesh);
	auto parent_hole = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(mesh, ATTRIBUTE_PARENT_HOLE_FACES);
	for (auto& face : mesh.face)
	{
		if (parent_hole[face] == 1)
			face.SetS();
	}

	//select only the vertices on the border of the interface
	vcg::tri::UpdateSelection<CMeshO>::VertexFromFaceStrict(mesh);
	vcg::tri::UpdateSelection<CMeshO>::FaceInvert(mesh);
	vcg::tri::UpdateSelection<CMeshO>::VertexFromFaceStrict(mesh, true);
	vcg::tri::UpdateSelection<CMeshO>::VertexInvert(mesh);
	vcg::tri::UpdateSelection<CMeshO>::FaceClear(mesh);

	for (auto& vertex : mesh.vert)
	{
		if (vertex.IsS()) {
			distance += vcg::Distance(vertex.cP(), tip.cP());
			count++;
		}
	}

	vcg::tri::UpdateSelection<CMeshO>::VertexClear(mesh);

	if (count > 0)
		return distance / count;
	else
		return 0;
}

Scalarm computeMeshArea(CMeshO& mesh)
{
	Scalarm area = 0;
	auto parent_hole = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(mesh, ATTRIBUTE_PARENT_HOLE_FACES);
	for (auto& face : mesh.face)
	{
		if (parent_hole[face] == 0) {
			area += vcg::DoubleArea(face);
		}
	}

	return area / 2.0;
}

Scalarm computeMeshVolume(CMeshO& mesh)
{
	return vcg::tri::Stat<CMeshO>::ComputeMeshVolume(mesh);
}

}
