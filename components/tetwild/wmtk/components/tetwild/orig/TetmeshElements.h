// This file is part of TetWild, a software for generating tetrahedral meshes.
//
// Copyright (C) 2018 Yixin Hu <yixin.hu@nyu.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//
// Created by yihu on 8/22/17.
//

#ifndef NEW_GTET_TETMESHELEMENTS_H
#define NEW_GTET_TETMESHELEMENTS_H

#include <unordered_set>
#include <wmtk/Types.hpp>
#include "State.h"

namespace wmtk::components::tetwild::orig {

class TetVertex
{
public:
    Vector3r pos; // todo: how to remove it?

    ///for surface conforming
    int on_fixed_vertex = -1;
    std::unordered_set<int> on_edge; // fixed points can be on more than one edges
    std::unordered_set<int> on_face;
    bool is_on_surface = false;

    ///for local operations
    std::unordered_set<int> conn_tets;

    ///for hybrid rationals
    Vector3d posf;
    bool is_rounded = false;

    void round()
    {
        // posf = Point_3f(CGAL::to_double(pos[0]), CGAL::to_double(pos[1]),
        // CGAL::to_double(pos[2]));
        posf = to_double(pos);
    }

    ///for bbox
    bool is_on_bbox = false;

    ///for boundary
    bool is_on_boundary = false;

    // for adaptive refinement
    double adaptive_scale = 1.0;

    TetVertex() = default;

    TetVertex(const Vector3r& p) { pos = p; }

    void printInfo() const;

    bool is_locked = false;
    bool is_inside = false;
};

class TetQuality
{
public:
    double slim_energy = 0;
    double volume = 0;

    TetQuality() = default;

    bool isBetterThan(const TetQuality& tq, int energy_type, const State& state)
    {
        return slim_energy < tq.slim_energy;
    }

    bool isBetterOrEqualThan(const TetQuality& tq, int energy_type, const State& state)
    {
        return slim_energy <= tq.slim_energy;
    }
};

} // namespace wmtk::components::tetwild::orig

#endif // NEW_GTET_TETMESHELEMENTS_H
