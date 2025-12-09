// This file is part of TetWild, a software for generating tetrahedral meshes.
//
// Copyright (C) 2018 Jeremie Dumas <jeremie.dumas@ens-lyon.org>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//
// Created by Jeremie Dumas on 09/04/18.
//

#include <igl/bounding_box_diagonal.h>

#include "Args.h"
#include "State.h"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::tetwild::orig {

State::State(const Args& args, const double& bbox_diagonal)
    : bbox_diag(bbox_diagonal)
    , eps_input(bbox_diag * args.eps_rel)
    , eps_delta(args.sampling_dist_rel > 0 ? 0 : eps_input / args.stage / std::sqrt(3))
    , initial_edge_len(args.getAbsoluteEdgeLength(bbox_diagonal))
    , bbox_dis(args.bbox_dis)
{
    if (args.sampling_dist_rel > 0) {
        // for testing only
        eps = bbox_diag * args.eps_rel;
        eps_2 = eps * eps;
        if (args.stage != 1) {
            log_and_throw_error("args.stage should be equal to 1.");
        }
    } else {
        eps = eps_input;
        eps_2 = eps * eps;
    }
}

} // namespace wmtk::components::tetwild::orig
