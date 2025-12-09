// This file is part of TetWild, a software for generating tetrahedral meshes.
//
// Copyright (C) 2018 Jeremie Dumas <jeremie.dumas@ens-lyon.org>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//
// Created by Jeremie Dumas on 09/10/18.
//

#include "TetmeshElements.h"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::tetwild::orig {

void TetVertex::printInfo() const
{
    logger().debug("is_on_surface = {}", is_on_surface);
    logger().debug("is_on_bbox = {}", is_on_bbox);
    logger().debug("conn_tets = {}", conn_tets);
}

} // namespace wmtk::components::tetwild::orig
