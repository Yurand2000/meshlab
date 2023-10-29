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

#include "PolylineCuttingFilter.h"

#include "common/PolylineMesh.h"
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/curve_on_manifold.h>
#include <vcg/complex/algorithms/outline_support.h>
#include <vcg/complex/algorithms/crease_cut.h>
#include <vcg/complex/algorithms/hole.h>
#include <vcg/complex/algorithms/isotropic_remeshing.h>
#include <vcg/space/fitting3.h>
#include <qregularexpression.h>

namespace curvatureSkeleton
{

struct refiningParameters;
using PolylineTags = std::pair<Scalarm, Scalarm>;
using Polylines = std::vector< std::pair<std::vector<vcg::Point3<Scalarm>>, PolylineTags> >;
using PolylineMeshes = std::vector< std::pair<PolylineMesh, PolylineTags> >;

static Polylines computePolylines(CMeshO& original, std::string const& facetag_id, vcg::CallBackPos* cb);
static PolylineMeshes convertToPolylineMeshes(Polylines& polylines);
static PolylineMeshes refinePolylines(Polylines const& polylines, PolylineMeshes&& polyline_meshes, CMeshO const& original, refiningParameters params, vcg::CallBackPos* cb);
static std::vector<PolylineMesh> cutPieces(FilterPlugin const& plugin, MeshDocument& document, CMeshO const& original, PolylineMeshes& polyline_meshes, std::string const& facetag_id, std::string const& holes_adj_facetag_id, bool close_holes, float refine_hole_lenght, vcg::CallBackPos* cb);
static void createPiecesMeshes(MeshDocument& document, MeshModel const& original_mm, CMeshO const& original, std::vector<PolylineMesh>& pieces, std::string const& facetag_id, std::string const& holes_adj_facetag_id);

static Scalarm getPolylineLenght(std::vector<vcg::Point3<Scalarm>> const& outlines);
static void movePolylineToFittingPlane(PolylineMesh& polyline, vcg::Plane3<Scalarm> const& plane, Scalarm weight);
static void movePolylinesApart(PolylineMesh& lpolyline, PolylineMesh const& rpolyline, Scalarm min_distance, Scalarm weight);
static Scalarm getMinVVDistance(PolylineMesh const& mesh0, PolylineMesh const& mesh1);
static void closeHoles(PolylineMesh& mesh, Scalarm refine_lenght);

struct refiningParameters {
	int number_of_iterations;
	float min_distance;
	float separation_weigth;
	float fit_plane_weigth;
	float smoothing_weigth;
	float projection_weigth;
};

std::map<std::string, QVariant> PolylineCuttingFilter::applyFilter(
	FilterPlugin const& plugin,
	RichParameterList const& params,
	MeshDocument& document,
	unsigned int&,
	vcg::CallBackPos* cb)
{
	//parameters
	auto* original_mm = document.mm();
	auto& original = original_mm->cm;
	auto facetag_id = params.getString(PARAM_FACE_TAG_ID).toStdString();
	auto generate_polylines = params.getBool(PARAM_GENERATE_POLYLINES);
	auto refine_polylines = params.getBool(PARAM_DO_REFINE_POLYLINES);
	auto close_holes = params.getBool(PARAM_CLOSE_HOLES);
	auto holes_adj_facetag_id = params.getString(PARAM_HOLE_ADJ_TAG_ID).toStdString();

	//refine parameters
	refiningParameters refining_params;
	refining_params.number_of_iterations = std::max(1, params.getInt(PARAM_REFINE_ITERATIONS));
	refining_params.smoothing_weigth = params.getDynamicFloat(PARAM_REFINE_SMOOTH_WEIGTH);
	refining_params.projection_weigth = params.getDynamicFloat(PARAM_REFINE_PROJECT_WEIGTH);
	refining_params.fit_plane_weigth = params.getDynamicFloat(PARAM_REFINE_FIT_PLANE_WEIGTH);
	refining_params.separation_weigth = params.getDynamicFloat(PARAM_REFINE_SEPARATION_WEIGHT);
	refining_params.min_distance = params.getAbsPerc(PARAM_REFINE_SEPARATION_MIN_DISTANCE);

	Scalarm refine_hole_lenght = 0;
	for (auto& face : original.face)
	{
		if (face.IsD()) continue;
		refine_hole_lenght +=
			vcg::Distance(face.V(0)->cP(), face.V(1)->cP()) +
			vcg::Distance(face.V(1)->cP(), face.V(2)->cP()) +
			vcg::Distance(face.V(2)->cP(), face.V(0)->cP());
	}

	if (original.FN() > 0)
		refine_hole_lenght /= (original.FN() * 3);

	if (original.face.empty())
		throw MLException("The given mesh has no faces.");

	vcg::tri::SelectionStack<CMeshO> selection(original);
	selection.push();
	{
		original.face.EnableFFAdjacency();
		vcg::tri::UpdateTopology<CMeshO>::FaceFace(original);

		auto polylines = computePolylines(original, facetag_id, cb);
		auto polyline_meshes = convertToPolylineMeshes(polylines);

		if (refine_polylines) {
			polyline_meshes = refinePolylines(polylines, std::move(polyline_meshes), original, refining_params, cb);
		}

		//generate polylines as meshes
		if (generate_polylines)
		{
			auto* polylines_mm = document.addNewMesh(QString(), QString("Polylines"), false);
			for (auto& polyline : polyline_meshes)
				vcg::tri::Append<CMeshO, PolylineMesh>::MeshAppendConst(polylines_mm->cm, polyline.first);
			polylines_mm->updateBoxAndNormals();
		}

		auto pieces = cutPieces(
			plugin, document, original, polyline_meshes, facetag_id, holes_adj_facetag_id,
			close_holes, refine_hole_lenght, cb
		);

		//for each new piece add a new mesh
		createPiecesMeshes(document, *original_mm, original, pieces, facetag_id, holes_adj_facetag_id);
	}
	selection.pop();
	return {};
}

Polylines computePolylines(
	CMeshO& original,
	std::string const& facetag_id,
	vcg::CallBackPos* cb
) {
	Polylines polylines;
	cb(0, "Computing polylines...");

	//clone original mesh;
	CMeshO mesh;
	auto facetag = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(mesh, facetag_id);
	vcg::tri::Append<CMeshO, CMeshO>::MeshCopyConst(mesh, original);

	mesh.face.EnableMark();
	mesh.face.EnableFFAdjacency();
	mesh.vert.EnableVFAdjacency();
	mesh.face.EnableVFAdjacency();
	vcg::tri::InitFaceIMark(mesh);

	//count how many different tags there are
	int different_tags = 0;
	{
		std::unordered_set<Scalarm> tags;
		for (auto& face : mesh.face)
			tags.emplace(facetag[face]);
		different_tags = tags.size();
	}

	different_tags -= 1; //number of iterations is at most the number of tags minus one.
	for (int i = 0; i < different_tags; i++)
	{
		cb(((i + 1) * 100.0 / different_tags) / 3, "Computing polylines...");

		//find tagged connected components
		auto facetag = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(mesh, facetag_id);
		std::unordered_map<Scalarm, std::vector<CFaceO*>> tagged_faces;
		for (auto& face : mesh.face)
		{
			auto tag = facetag[face];
			if (tagged_faces.count(tag) > 0)
				tagged_faces[tag].emplace_back(&face);
			else
				tagged_faces.insert({ tag, {&face} });
		}
		tagged_faces.erase(-1);

		//get the connected component that has fewer holes, in case of parity pick the smallest in polylines lenght
		vcg::tri::UpdateTopology<CMeshO>::FaceFace(mesh);
		vcg::tri::UpdateTopology<CMeshO>::VertexFace(mesh);

		Scalarm best_tag = -1; Scalarm best_outlines_total_lenght = 0;
		std::vector< std::vector<vcg::Point3<Scalarm>> > best_outlines;
		for (auto& pair : tagged_faces)
		{
			auto tag = pair.first;
			auto& faces = pair.second;

			vcg::tri::UpdateSelection<CMeshO>::FaceClear(mesh);
			vcg::tri::UpdateSelection<CMeshO>::VertexClear(mesh);
			for (auto* face : faces)
				face->SetS();

			//get borders
			CMeshO tagged_component;
			vcg::tri::Append<CMeshO, CMeshO>::MeshCopy(tagged_component, mesh, true);

			tagged_component.face.EnableFFAdjacency();
			vcg::tri::UpdateTopology<CMeshO>::FaceFace(tagged_component);
			std::vector< std::vector<vcg::Point3<Scalarm>> > outlines;
			vcg::tri::OutlineUtil<Scalarm>::ConvertMeshBoundaryToOutline3Vec(tagged_component, outlines);

			Scalarm outlines_total_lenght = 0;
			for (auto& outline : outlines)
				outlines_total_lenght += getPolylineLenght(outline);

			//save borders only if less than the best found borders
			if (best_tag == -1 || outlines.size() < best_outlines.size() ||
				(outlines.size() == best_outlines.size() && outlines_total_lenght < best_outlines_total_lenght)
				) {
				outlines.swap(best_outlines);
				best_tag = tag;
				best_outlines_total_lenght = outlines_total_lenght;
			}
		}

		//get which neighbor has the most faces adjacent on the currently selected tag on the original mesh
		Scalarm neighbor_with_most_adjacent_faces = -1;
		{
			//select border faces (the ones with tag == -1)
			auto facetag = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(original, facetag_id);
			vcg::tri::UpdateSelection<CMeshO>::FaceClear(original);
			for (auto& face : tagged_faces[best_tag])
				original.face[face->Index()].SetS();
			vcg::tri::UpdateSelection<CMeshO>::FaceDilate(original);
			for (auto* face : tagged_faces[best_tag])
				original.face[face->Index()].ClearS();

			//compute biggest neighbor
			std::unordered_map<Scalarm, int> neighbor_tag_count;
			Scalarm best_count = 0;
			for (auto& face : original.face)
			{
				if (!face.IsS())
					continue;

				std::vector<CFaceO*> star;
				vcg::face::FFExtendedStarFF(&face, 1, star);

				for (auto* adj : star)
				{
					auto adj_tag = facetag[adj];
					if (adj_tag != -1 && adj_tag != best_tag)
					{
						if (neighbor_tag_count.count(adj_tag) > 0)
							neighbor_tag_count[adj_tag] += 1;
						else
							neighbor_tag_count[adj_tag] = 0;

						auto count = neighbor_tag_count[adj_tag];
						if (count > best_count) {
							best_count = count;
							neighbor_with_most_adjacent_faces = adj_tag;
						}
					}
				}
			}
		}

		//add the polylines to the polyline set
		for (int i = 0; i < best_outlines.size(); i++) {
			polylines.emplace_back(
				std::move(best_outlines[i]),
				std::make_pair(best_tag, neighbor_with_most_adjacent_faces)
			);
		}

		//select border of untagged faces
		std::vector<CFaceO*> notag_faces;
		std::queue<CFaceO*> frontier;
		vcg::tri::UnMarkAll(mesh);
		for (auto& face : tagged_faces[best_tag]) {
			frontier.push(face);
			vcg::tri::Mark(mesh, face);
		}

		while (!frontier.empty())
		{
			auto* face = frontier.front(); frontier.pop();

			std::vector<CFaceO*> star;
			vcg::face::FFExtendedStarFF(face, 1, star);
			for (auto* adj : star)
			{
				if (vcg::tri::IsMarked(mesh, adj))
					continue;

				vcg::tri::Mark(mesh, adj);
				if (facetag[adj] == -1)
				{
					notag_faces.push_back(adj);
					frontier.push(adj);
				}
			}
		}

		//set the current connected component to the neighbors with most adj faces
		for (auto& face : tagged_faces[best_tag])
			facetag[face] = neighbor_with_most_adjacent_faces;

		int last_size = -1;
		while (notag_faces.size() != last_size)
		{
			last_size = notag_faces.size();
			std::vector<CFaceO*> new_notag_faces;
			for (auto* face : notag_faces)
			{
				bool has_current_neighbor = false;
				bool has_other_neighbors = false;

				std::unordered_set<CFaceO*> star;
				for (int i = 0; i < 3; i++) {
					std::vector<CFaceO*> vstar; std::vector<int> indices;
					vcg::face::VFStarVF(face->V(i), vstar, indices);
					for (auto* face : vstar)
						star.emplace(face);
				}

				for (auto* adj : star)
				{
					if (facetag[adj] != -1 && facetag[adj] != neighbor_with_most_adjacent_faces)
						has_other_neighbors = true;

					if (facetag[adj] == neighbor_with_most_adjacent_faces)
						has_current_neighbor = true;
				}

				if (!has_other_neighbors) {
					if (!has_current_neighbor)
						new_notag_faces.push_back(face);
					else
						facetag[face] = neighbor_with_most_adjacent_faces;
				}
			}

			new_notag_faces.swap(notag_faces);
		}
	}

	return polylines;
}

PolylineMeshes convertToPolylineMeshes(Polylines& polylines) {
	PolylineMeshes polyline_meshes(polylines.size());
	for (int i = 0; i < polylines.size(); i++)
	{
		auto& polyline_vec = polylines[i].first;
		auto& polyline = polyline_meshes[i].first;
		polyline_meshes[i].second = polylines[i].second;
		vcg::tri::OutlineUtil<Scalarm>::ConvertOutline3VecToEdgeMesh(polyline_vec, polyline);
	}

	return polyline_meshes;
}

PolylineMeshes refinePolylines(
	Polylines const& polylines,
	PolylineMeshes&& polyline_meshes,
	CMeshO const& original,
	refiningParameters params,
	vcg::CallBackPos* cb
) {
	cb(33, "Refining polylines...");
	std::vector< std::pair<PolylineMesh, vcg::Plane3<Scalarm>> > single_polylines(polylines.size());

	for (int i = 0; i < polyline_meshes.size(); i++)
		std::swap(single_polylines[i].first, polyline_meshes[i].first);

	PolylineMesh c_original;
	vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopyConst(c_original, original);
	auto com = vcg::tri::CoM<PolylineMesh>(c_original);
	com.Init();

	//generate fitting planes
	for (int i = 0; i < polylines.size(); i++)
	{
		auto& polyline_vec = polylines[i].first;
		auto& fitting_plane = single_polylines[i].second;
		vcg::FitPlaneToPointSet(polyline_vec, fitting_plane);
	}

	//smooth-project
	for (int iter = 0; iter < params.number_of_iterations; iter++)
	{
		cb(((iter + 1) * 100 / params.number_of_iterations) / 3 + (100 / 3), "Refining polylines...");
		for (auto& pair : single_polylines)
		{
			auto& polyline = pair.first;
			auto& plane = pair.second;

			//smooth project polyline
			if (polyline.EN() > 0)
			{
				for (auto& pair2 : single_polylines) {
					auto& polyline2 = pair2.first;
					if (&polyline != &polyline2)
						movePolylinesApart(polyline, polyline2, params.min_distance, params.separation_weigth);
				}

				movePolylineToFittingPlane(polyline, plane, params.fit_plane_weigth);
				com.SmoothProject(polyline, 1, params.smoothing_weigth, params.projection_weigth);
				vcg::tri::Clean<PolylineMesh>::RemoveUnreferencedVertex(polyline);
				vcg::tri::Allocator<PolylineMesh>::CompactEveryVector(polyline);
			}
		}
	}

	for (int i = 0; i < single_polylines.size(); i++)
		std::swap(polyline_meshes[i].first, single_polylines[i].first);

	return polyline_meshes;
}

std::vector<PolylineMesh> cutPieces(
	FilterPlugin const& plugin,
	MeshDocument& document,
	CMeshO const& original,
	PolylineMeshes& polyline_meshes,
	std::string const& facetag_id,
	std::string const& holes_adj_facetag_id,
	bool close_holes,
	float refine_hole_lenght,
	vcg::CallBackPos* cb
) {
	std::vector<PolylineMesh> pieces;
	int current_polyline = 0;
	int num_polylines = polyline_meshes.size();

	cb(66, "Cutting polylines...");
	std::queue< std::pair<PolylineMesh, std::vector<std::pair<PolylineMesh, PolylineTags>>> > split_queue;

	//first split-queue pair is the main mesh and all the (valid) polylines
	split_queue.emplace();
	PolylineMesh& c_original = split_queue.back().first;
	vcg::tri::Allocator<PolylineMesh>::AddPerFaceAttribute<Scalarm>(c_original, facetag_id);
	vcg::tri::Append<PolylineMesh, CMeshO>::MeshCopyConst(c_original, original);

	auto holes_adj = vcg::tri::Allocator<PolylineMesh>::GetPerFaceAttribute<Scalarm>(c_original, holes_adj_facetag_id);
	for (auto& face : c_original.face)
		holes_adj[face] = -1;

	split_queue.back().second.swap(polyline_meshes);

	if (num_polylines == 0) {
		pieces.emplace_back();
		std::swap(pieces.back(), split_queue.back().first);
		return pieces;
	}

	//foreach mesh polylines pair
	while (!split_queue.empty())
	{
		cb(((current_polyline + 1) * 100 / num_polylines) / 3 + (200 / 3), "Cutting polylines...");

		PolylineMesh mesh;
		std::vector<std::pair<PolylineMesh, PolylineTags>> polylines;
		std::swap(split_queue.front().first, mesh);
		std::swap(split_queue.front().second, polylines);
		current_polyline++;
		split_queue.pop();

		//if no polylines to cut we are done
		if (polylines.empty())
		{
			pieces.emplace_back();
			std::swap(pieces.back(), mesh);
		}
		else //cut on any polyline
		{
			auto polyline = std::move(polylines.back().first);
			auto polyline_adj_tags = std::move(polylines.back().second);
			polylines.pop_back();

			//prepare curve on manifold for the mesh
			vcg::tri::UpdateTopology<PolylineMesh>::FaceFace(mesh);

			try
			{
				vcg::tri::MeshAssert<PolylineMesh>::FFTwoManifoldEdge(mesh);
			}
			catch (std::runtime_error e)
			{
				plugin.log(e.what());
				auto error_mesh = document.addNewMesh("", "Error Branch", false);
				vcg::tri::Append<CMeshO, PolylineMesh>::MeshCopyConst(error_mesh->cm, mesh);
				error_mesh->updateBoxAndNormals();
				auto error_poly = document.addNewMesh("", "Error Polyline", false);
				vcg::tri::Append<CMeshO, PolylineMesh>::MeshCopyConst(error_poly->cm, polyline);
				error_poly->updateBoxAndNormals();
				break;
			}

			vcg::tri::UpdateTopology<PolylineMesh>::VertexFace(mesh);
			vcg::tri::UpdateTopology<PolylineMesh>::VertexEdge(mesh);
			vcg::tri::UpdateSelection<PolylineMesh>::Clear(mesh);
			auto com = vcg::tri::CoM<PolylineMesh>(mesh);
			com.Init();

			//refine the mesh on the polyline
			com.RefineCurveByBaseMesh(polyline);
			com.SplitMeshWithPolyline(polyline);
			com.TagFaceEdgeSelWithPolyLine(polyline);
			vcg::tri::CutMeshAlongSelectedFaceEdges(mesh);

			//get the two connected components of the cut mesh
			std::vector<std::pair<int, PolylineFace*>> conn_comps;
			vcg::tri::UpdateTopology<PolylineMesh>::FaceFace(mesh);
			vcg::tri::Clean<PolylineMesh>::ConnectedComponents(mesh, conn_comps);

			int index0 = 0, index1 = 1;
			if (conn_comps.size() > 2)
			{
				throw MLException("More than 2 connected components on single polyline!");
			}
			else if (conn_comps.size() < 2) {
				vcg::tri::UpdateSelection<PolylineMesh>::Clear(mesh);

				split_queue.emplace();
				std::swap(split_queue.back().first, mesh);
				std::swap(split_queue.back().second, polylines);
			}
			else
			{
				PolylineMesh part0, part1;

				//keep face tags on each piece
				vcg::tri::Allocator<PolylineMesh>::AddPerFaceAttribute<Scalarm>(part0, facetag_id);
				vcg::tri::Allocator<PolylineMesh>::AddPerFaceAttribute<Scalarm>(part0, holes_adj_facetag_id);
				vcg::tri::Allocator<PolylineMesh>::AddPerFaceAttribute<Scalarm>(part1, facetag_id);
				vcg::tri::Allocator<PolylineMesh>::AddPerFaceAttribute<Scalarm>(part1, holes_adj_facetag_id);

				conn_comps[index0].second->SetS();
				vcg::tri::UpdateSelection<PolylineMesh>::FaceConnectedFF(mesh);
				vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshCopy(part0, mesh, true);
				vcg::tri::UpdateSelection<PolylineMesh>::Clear(part0);
				vcg::tri::UpdateSelection<PolylineMesh>::Clear(mesh);

				conn_comps[index1].second->SetS();
				vcg::tri::UpdateSelection<PolylineMesh>::FaceConnectedFF(mesh);
				vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshCopy(part1, mesh, true);
				vcg::tri::UpdateSelection<PolylineMesh>::Clear(part1);
				vcg::tri::UpdateSelection<PolylineMesh>::Clear(mesh);

				//compute which of the remaining polylines cut the part 0 and which the part 1
				std::vector<std::pair<PolylineMesh, PolylineTags>> part0_polys, part1_polys;

				for (int i = 0; i < polylines.size(); i++)
				{
					auto& polyline = polylines[i];
					auto distance0 = getMinVVDistance(polyline.first, part0);
					auto distance1 = getMinVVDistance(polyline.first, part1);
					if (distance0 < distance1)
					{
						part0_polys.emplace_back();
						std::swap(polyline, part0_polys.back());
					}
					else
					{
						part1_polys.emplace_back();
						std::swap(polyline, part1_polys.back());
					}
				}

				if (close_holes)
				{
					//compute which part has generated the polyline (needed to set the adjacency tag for the closed holes)
					auto part0_adj_tag = polyline_adj_tags.first;
					auto part1_adj_tag = polyline_adj_tags.second;

					{
						int part0count = 0, part1count = 0;
						auto facetag0 = vcg::tri::Allocator<PolylineMesh>::GetPerFaceAttribute<Scalarm>(part0, facetag_id);
						for (auto& face : part0.face)
							if (facetag0[face] == part0_adj_tag)
								part0count++;

						auto facetag1 = vcg::tri::Allocator<PolylineMesh>::GetPerFaceAttribute<Scalarm>(part1, facetag_id);
						for (auto& face : part1.face)
							if (facetag1[face] == part0_adj_tag)
								part1count++;

						if (part0count > part1count)
							std::swap(part0_adj_tag, part1_adj_tag);
					}

					//close hole on the first piece
					closeHoles(part0, refine_hole_lenght);

					//duplicate the cap onto the other mesh and merge close vertices
					vcg::tri::UpdateSelection<PolylineMesh>::VertexFromFaceLoose(part0);
					vcg::tri::Append<PolylineMesh, PolylineMesh>::MeshAppendConst(part1, part0, true);
					vcg::tri::Clean<PolylineMesh>::FlipMesh(part1, true);
					vcg::tri::Clean<PolylineMesh>::MergeCloseVertex(part1, 1.0E-7);

					//update the hole adjacency tags
					auto adj_facetag0 = vcg::tri::Allocator<PolylineMesh>::GetPerFaceAttribute<Scalarm>(part0, holes_adj_facetag_id);
					for (auto& face : part0.face) {
						if (face.IsS())
							adj_facetag0[face] = part0_adj_tag;
						else if (adj_facetag0[face] == 0)
							adj_facetag0[face] = -1;
					}

					auto adj_facetag1 = vcg::tri::Allocator<PolylineMesh>::GetPerFaceAttribute<Scalarm>(part1, holes_adj_facetag_id);
					for (auto& face : part1.face) {
						if (face.IsS())
							adj_facetag1[face] = part1_adj_tag;
						else if (adj_facetag1[face] == 0)
							adj_facetag1[face] = -1;
					}
				}

				//add the new pieces to the queue
				vcg::tri::UpdateSelection<PolylineMesh>::Clear(part0);
				vcg::tri::UpdateSelection<PolylineMesh>::Clear(part1);

				split_queue.emplace();
				std::swap(split_queue.back().first, part0);
				std::swap(split_queue.back().second, part0_polys);

				split_queue.emplace();
				std::swap(split_queue.back().first, part1);
				std::swap(split_queue.back().second, part1_polys);
			}
		}
	}

	return pieces;
}

void createPiecesMeshes(
	MeshDocument& document,
	MeshModel const& original_mm,
	CMeshO const& original,
	std::vector<PolylineMesh>& pieces,
	std::string const& facetag_id,
	std::string const& holes_adj_facetag_id
) {
	for (int i = 0; i < pieces.size(); i++)
	{
		auto original_name = original_mm.label().remove(QRegularExpression("\\.\\w+$"));

		//add the new mesh
		auto* piece_mm = document.addNewMesh(QString(), QString(), false);
		if (original.face.IsColorEnabled())
		{
			piece_mm->updateDataMask(piece_mm->MM_FACECOLOR);
			piece_mm->cm.face.EnableColor();
		}
		if (original_mm.hasPerVertexColor())
			piece_mm->updateDataMask(piece_mm->MM_VERTCOLOR);
		vcg::tri::Allocator<CMeshO>::AddPerFaceAttribute<Scalarm>(piece_mm->cm, facetag_id);
		vcg::tri::Allocator<CMeshO>::AddPerFaceAttribute<Scalarm>(piece_mm->cm, holes_adj_facetag_id);
		vcg::tri::Append<CMeshO, PolylineMesh>::MeshAppendConst(piece_mm->cm, pieces[i]);
		piece_mm->updateBoxAndNormals();

		//find the most occurring tag of the piece and set it as label
		Scalarm max_tag = 0;
		{
			std::unordered_map<int, int> tag_count; int max_count = 0;
			auto facetag = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(piece_mm->cm, facetag_id);
			for (auto& face : piece_mm->cm.face)
			{
				auto tag = facetag[face];
				if (tag_count.count(tag) > 0)
					tag_count[tag]++;
				else
					tag_count.emplace(tag, 0);

				if (tag > 0 && tag_count[tag] > max_count)
				{
					max_tag = tag;
					max_count = tag_count[tag];
				}
			}
		}
		piece_mm->setLabel(QString("%3 - Part #%1; Tag %2").arg(i).arg(max_tag).arg(original_name));
		piece_mm->updateBoxAndNormals();
	}
}

void movePolylinesApart(PolylineMesh& lpolyline, PolylineMesh const& rpolyline, Scalarm min_distance, Scalarm weight)
{
	for (auto& lvert : lpolyline.vert) {
		if (lvert.IsD()) continue;
		for (auto& redge : rpolyline.edge) {
			if (redge.IsD()) continue;

			Scalarm distance = min_distance;
			vcg::Point3<Scalarm> proj_point;
			if (vcg::edge::PointDistance(redge, lvert.cP(), distance, proj_point)) {
				auto difference_vec = lvert.cP() - proj_point;
				lvert.P() += difference_vec.normalized() * std::fmax(min_distance - distance, 0.0) * weight;
			}
		}
	}
}

void movePolylineToFittingPlane(PolylineMesh& polyline, vcg::Plane3<Scalarm> const& plane, Scalarm weight)
{
	for (auto& vert : polyline.vert)
	{
		auto proj_point = plane.Projection(vert.cP());
		auto difference_vec = proj_point - vert.cP();
		vert.P() += difference_vec * weight;
	}
}

Scalarm getMinVVDistance(PolylineMesh const& mesh0, PolylineMesh const& mesh1)
{
	Scalarm min_dist = std::numeric_limits<Scalarm>::max();

	for (auto& vert0 : mesh0.vert)
	{
		for (auto& vert1 : mesh1.vert)
		{
			auto distance = vcg::SquaredDistance(vert0.cP(), vert1.cP());
			if (distance < min_dist)
				min_dist = distance;
		}
	}

	min_dist = vcg::math::Sqrt(min_dist);
	return min_dist;
}

void closeHoles(PolylineMesh& mesh, Scalarm refine_lenght)
{
	//code adapted from meshfilter.cpp

	size_t original_faces_count = mesh.face.size();
	auto max_hole_size = 50000;

	int holes_closed = 0; int stack_size = 0;
	vcg::tri::SelectionStack<PolylineMesh> selection(mesh);
	vcg::tri::UpdateSelection<PolylineMesh>::Clear(mesh);
	do
	{
		//close holes
		vcg::tri::UpdateTopology<PolylineMesh>::FaceFace(mesh);
		holes_closed = vcg::tri::Hole<PolylineMesh>::EarCuttingFill< vcg::tri::MinimumWeightEar<PolylineMesh> >(mesh, max_hole_size, false, &vcg::DummyCallBackPos);

		//select newly added faces
		vcg::tri::UpdateSelection<PolylineMesh>::FaceClear(mesh);
		for (int i = original_faces_count; i < mesh.face.size(); i++)
		{
			auto& face = mesh.face[i];
			if (!face.IsD())
				face.SetS();
		}

		//save selection in stack to restore it later
		selection.push();
		stack_size++;
	} while (holes_closed != 0);

	//select all the newly added faces
	for (int i = 0; i < stack_size; i++)
		selection.popOr();

	//refine newly added faces
	vcg::tri::IsotropicRemeshing<PolylineMesh>::Params params;
	params.SetFeatureAngleDeg(181.0f);
	params.adapt = false;
	params.selectedOnly = true;
	params.splitFlag = true;
	params.collapseFlag = true;
	params.swapFlag = true;
	params.smoothFlag = true;
	params.projectFlag = false;
	params.surfDistCheck = false;

	for (int k = 0; k < 3; k++)
	{
		params.SetTargetLen(refine_lenght * 3.0); params.iter = 5;
		vcg::tri::IsotropicRemeshing<PolylineMesh>::Do(mesh, params);

		params.SetTargetLen(refine_lenght / 3.0); params.iter = 3;
		vcg::tri::IsotropicRemeshing<PolylineMesh>::Do(mesh, params);

		params.SetTargetLen(refine_lenght); params.iter = 2;
		vcg::tri::IsotropicRemeshing<PolylineMesh>::Do(mesh, params);
	}
}

Scalarm getPolylineLenght(std::vector<vcg::Point3<Scalarm>> const& outlines)
{
	Scalarm lenght = 0;
	lenght += vcg::Distance(outlines[0], outlines[outlines.size() - 1]);
	for (int i = 1; i < outlines.size(); i++)
		lenght += vcg::Distance(outlines[i-1], outlines[i]);

	return lenght;
}

}
