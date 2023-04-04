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

#include "RestoreHackOrderDataFilter.h"

#include <qregularexpression.h>

#define ATTRIBUTE_ORIGINAL_INDEX "original_index"
#define ATTRIBUTE_HACK_ORDER_NUMBER "hack_order"
#define ATTRIBUTE_PARENT_HOLE_FACES "parent_hole"

namespace curvatureSkeleton
{

std::map<std::string, QVariant> RestoreHackOrderDataFilter::applyFilter(
	FilterPlugin const&      plugin,
	RichParameterList const& params,
	MeshDocument&            document,
	unsigned int&,
	vcg::CallBackPos* cb)
{
	auto  original_mm = document.getMesh(params.getMeshId(PARAM_ORIGINAL_MESH));
	auto& original = original_mm->cm;

	auto facetag_id = params.getString(PARAM_FACE_TAG_ID);
	auto holes_adj_facetag_id = params.getString(PARAM_HOLE_ADJ_TAG_ID);

	//build map from face tag to hack order
	std::unordered_map<int, int> tag_to_hack_order;
	{
		auto facetag = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(original, facetag_id.toStdString());
		auto faceorder = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(original, ATTRIBUTE_HACK_ORDER_NUMBER);
		for (auto& face : original.face)
		{
			auto tag = facetag[face];
			if (tag_to_hack_order.count(tag) == 0)
				tag_to_hack_order.emplace(tag, faceorder[face]);
		}
	}
	tag_to_hack_order.erase(-1);

	//foreach visibile mesh
	std::unordered_map<int, std::pair<int, MeshModel*>> mesh_tags;
	{
		auto regex = QRegularExpression(QString("Piece\\s+#(\\d+);\\s+Tag\\s+(\\d+)"));
		for (auto& mesh_mm : document.meshIterator())
		{
			if (!mesh_mm.isVisible() || mesh_mm.id() == original_mm->id())
				continue;

			//get the mesh tag from the mesh label
			auto match = regex.match(mesh_mm.label());
			if (match.hasMatch()) {
				mesh_tags.emplace(match.captured(2).toInt(), std::make_pair( match.captured(1).toInt(), &mesh_mm ));
			}
		}
	}

	//foreach visibile mesh
	for (auto& pair : mesh_tags)
	{
		auto tag = pair.first;
		auto piece_id = pair.second.first;
		auto* mesh_mm = pair.second.second;
		auto& mesh = mesh_mm->cm;

		//find tag of adjacent parent
		int parent_tag = tag;
		{
			auto adj_facetag = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(mesh, holes_adj_facetag_id.toStdString());
			int parent_order = tag_to_hack_order[tag];
			for (auto& face : mesh.face)
			{
				auto tag = adj_facetag[face];
				if (tag == -1)
					continue;

				auto order = tag_to_hack_order[tag];
				if (order < parent_order) {
					parent_order = order;
					parent_tag = tag;
				}
			}
		}

		//update the label with the hack order and parent numbers
		if (parent_tag != tag) {
			mesh_mm->setLabel(QString("Piece #%1; Hack %2; Parent: #%3")
				.arg(piece_id).arg(tag_to_hack_order[tag]).arg(mesh_tags[parent_tag].first)
			);
		}
		else {
			mesh_mm->setLabel(QString("Piece #%1; Hack %2")
				.arg(piece_id).arg(tag_to_hack_order[tag])
			);
		}

		//assign the hack numbers to each face given its tag
		{
			auto facetag = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(mesh, facetag_id.toStdString());
			auto faceorder = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(mesh, ATTRIBUTE_HACK_ORDER_NUMBER);
			for (auto& face : mesh.face)
			{
				auto tag = facetag[face];
				faceorder[face] = tag_to_hack_order[tag];
			}
		}

		//get which of the close hole surfaces is adjacent to the parent
		{
			auto adj_facetag = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(mesh, holes_adj_facetag_id.toStdString());
			auto parent_hole_tag = vcg::tri::Allocator<CMeshO>::GetPerFaceAttribute<Scalarm>(mesh, ATTRIBUTE_PARENT_HOLE_FACES);
			for (auto& face : mesh.face)
			{
				auto tag = adj_facetag[face];
				if (tag == parent_tag)
					parent_hole_tag[face] = 1;
				else
					parent_hole_tag[face] = 0;
			}
		}
	}

	return {};
}

}
