// This file is part of TetWild, a software for generating tetrahedral meshes.
//
// Copyright (C) 2018 Yixin Hu <yixin.hu@nyu.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//
// Created by Yixin Hu on 4/11/17.
//

#ifndef NEW_GTET_MESHREFINEMENT_H
#define NEW_GTET_MESHREFINEMENT_H

#include <igl/Timer.h>

#include "ForwardDecls.h"
#include "TetmeshElements.h"

namespace wmtk::components::tetwild::orig {

class MeshRefinement
{
public:
    const Args& args;
    State& state;

    SampleEnvelope& env_sf; // surface envelope
    SampleEnvelope& env_b; // boundary envelope

    // init
    std::vector<TetVertex> tet_vertices;
    std::vector<std::array<int, 4>> tets;
    // prepare data
    std::vector<bool> v_is_removed;
    std::vector<bool> t_is_removed;
    std::vector<TetQuality> tet_qualities;
    std::vector<std::array<int, 4>> is_surface_fs;

    igl::Timer igl_timer;

    int old_pass = 0;

    MeshRefinement(SampleEnvelope& env_surf, SampleEnvelope& env_bound, const Args& ar, State& st)
        : env_sf(env_surf)
        , env_b(env_bound)
        , args(ar)
        , state(st)
    {}

    void prepareData(bool is_init = true);
    void round();
    void clear();

    int sf_id = 0;
    int doOperations(
        EdgeSplitter& splitter,
        EdgeCollapser& collapser,
        EdgeRemover& edge_remover,
        VertexSmoother& smoother,
        const std::array<bool, 4>& ops = {{true, true, true, true}});
    int doOperationLoops(
        EdgeSplitter& splitter,
        EdgeCollapser& collapser,
        EdgeRemover& edge_remover,
        VertexSmoother& smoother,
        int max_pass,
        const std::array<bool, 4>& ops = {{true, true, true, true}});
    bool is_dealing_unrounded = false;
    bool is_dealing_local = false;

    void refine(
        const std::array<bool, 4>& ops = {{true, true, true, true}},
        bool is_pre = true,
        bool is_post = true,
        int scalar_update = 3);
    void refine_pre(
        EdgeSplitter& splitter,
        EdgeCollapser& collapser,
        EdgeRemover& edge_remover,
        VertexSmoother& smoother);
    void refine_post(
        EdgeSplitter& splitter,
        EdgeCollapser& collapser,
        EdgeRemover& edge_remover,
        VertexSmoother& smoother);

    double min_adaptive_scale;
    bool is_hit_min = false;
    void updateScalarField(
        bool is_clean_up_unrounded,
        bool is_clean_up_local,
        double filter_energy,
        bool is_lock = false);

    int getInsideVertexSize();
    void markInOut(std::vector<bool>& tmp_t_is_removed);

    int mid_result = 0;
    void getSurface(Eigen::MatrixXd& V, Eigen::MatrixXi& F);
    void getTrackedSurface(Eigen::MatrixXd& V, Eigen::MatrixXi& F);
    void getTrackedSurface_continuous(Eigen::MatrixXd& V, Eigen::MatrixXi& F);
};

} // namespace wmtk::components::tetwild::orig

#endif // NEW_GTET_MESHREFINEMENT_H
