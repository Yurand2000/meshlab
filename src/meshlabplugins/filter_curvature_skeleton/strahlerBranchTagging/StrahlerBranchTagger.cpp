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

#include "StrahlerBranchTagger.h"
#include "StrahlerBranchTagger_private.h"

namespace curvatureSkeleton
{
    
void StrahlerBranchTagger::calculateStrahlerNumbers(CMeshO& original, CMeshO& skeleton, CMeshO& tree)
{
	StrahlerNumberCalculator(tree).compute();
	SaveStrahlerNumber(original, skeleton, tree).saveNumbers();
}

StrahlerBranchTagger::StrahlerNodeNumbers StrahlerBranchTagger::getNodeNumbers(CMeshO const& tree_mesh)
{
	return StrahlerNumberCalculator::getNodeNumbers(tree_mesh);
}

StrahlerBranchTagger::StrahlerBranchNumbers StrahlerBranchTagger::getBranchNumbers(CMeshO const& tree_mesh)
{
	return StrahlerNumberCalculator::getBranchNumbers(tree_mesh);
}

void StrahlerBranchTagger::strahlerNumberToQuality(CMeshO& mesh)
{
	SaveStrahlerNumber::strahlerNumberToQuality(mesh);
}

}
