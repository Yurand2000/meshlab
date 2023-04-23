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

#include "Utils.h"

#include <vcg/space/colormap.h>

namespace curvatureSkeleton
{
    bool MeshDocumentUtils::tryGetOriginalMeshIndex(MeshDocument const& document, int& out)
    {
        auto skeleton_id = 0;
        if (document.meshNumber() == 2 && tryGetSkeletonMeshIndex(document, skeleton_id))
        {
            for (auto& mesh : document.meshIterator())
            {
                if (mesh.id() != skeleton_id) {
                    out = mesh.id();
                    return true;
                }
            }

            return false;
        }
        else
        {
            return false;
        }
    }

    bool MeshDocumentUtils::tryGetSkeletonMeshIndex(MeshDocument const& document, int& out)
    {
        for (auto& mesh : document.meshIterator())
        {
            if (mesh.label() != nullptr && mesh.label().contains("skel", Qt::CaseSensitivity::CaseInsensitive))
            {
                out = mesh.id();
                return true;
            }
        }
        return false;
    }

    vcg::Color4b HackOrderUtils::hackOrderToColor(int hack_order_number)
    {
        return vcg::GetColorMapping((hack_order_number - 1) % 6, 0, 5, vcg::Plasma);
    }
}
