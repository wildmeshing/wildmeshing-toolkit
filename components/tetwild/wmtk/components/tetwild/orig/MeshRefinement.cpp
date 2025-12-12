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

#include <igl/winding_number.h>

#include <wmtk/utils/DisableWarnings.hpp>

#include "Args.h"
#include "Common.h"
#include "EdgeCollapser.h"
#include "EdgeRemover.h"
#include "EdgeSplitter.h"
#include "MeshRefinement.h"
#include "VertexSmoother.h"

#include <wmtk/utils/EnableWarnings.hpp>

#include <SimpleBVH/BVH.hpp>
#include <wmtk/envelope/KNN.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::tetwild::orig {

void MeshRefinement::prepareData(bool is_init)
{
    igl_timer.start();
    if (is_init) {
        t_is_removed = std::vector<bool>(tets.size(), false); // have to
        v_is_removed = std::vector<bool>(tet_vertices.size(), false);
        for (int i = 0; i < tet_vertices.size(); i++) {
            if (tet_vertices[i].is_rounded) continue;
            tet_vertices[i].round();
        }
        round();
    }

    SampleEnvelope env;

    LocalOperations localOperation(
        tet_vertices,
        tets,
        is_surface_fs,
        v_is_removed,
        t_is_removed,
        tet_qualities,
        env,
        env,
        args,
        state);
    localOperation.calTetQualities(tets, tet_qualities);
    double tmp_time = igl_timer.getElapsedTime();
    logger().debug("{}s", tmp_time);
    localOperation.outputInfo(MeshRecord::OpType::OP_OPT_INIT, tmp_time);
}

void MeshRefinement::round()
{
    int cnt = 0;
    int sub_cnt = 0;
    for (int i = 0; i < tet_vertices.size(); i++) {
        if (v_is_removed[i]) continue;
        if (tet_vertices[i].is_rounded) continue;
        tet_vertices[i].is_rounded = true;
        Vector3r old_p = tet_vertices[i].pos;
        tet_vertices[i].pos = to_rational(tet_vertices[i].posf);

        for (auto it = tet_vertices[i].conn_tets.begin(); it != tet_vertices[i].conn_tets.end();
             it++) {
            Vector3r n =
                ((tet_vertices[tets[*it][1]].pos) - tet_vertices[tets[*it][0]].pos)
                    .cross((tet_vertices[tets[*it][2]].pos) - tet_vertices[tets[*it][0]].pos);
            Vector3r d = (tet_vertices[tets[*it][3]].pos) - tet_vertices[tets[*it][0]].pos;
            auto res = n.dot(d);

            if (res <= 0) {
                tet_vertices[i].is_rounded = false;
                break;
            }
        }
        if (!tet_vertices[i].is_rounded)
            tet_vertices[i].pos = old_p;
        else {
            cnt++;
            sub_cnt++;
        }
    }
    logger().debug("round: {}({})", cnt, tet_vertices.size());
}

void MeshRefinement::clear()
{
    tet_vertices.clear();
    tets.clear();

    t_is_removed.clear();
    v_is_removed.clear();
    is_surface_fs.clear();
    tet_qualities.clear();
}

int MeshRefinement::doOperations(
    EdgeSplitter& splitter,
    EdgeCollapser& collapser,
    EdgeRemover& edge_remover,
    VertexSmoother& smoother,
    const std::array<bool, 4>& ops)
{
    int cnt0 = 0;
    for (int i = 0; i < tet_vertices.size(); i++) {
        if (v_is_removed[i] || tet_vertices[i].is_locked || tet_vertices[i].is_rounded) continue;
        cnt0++;
    }
    bool is_log = true;
    double tmp_time;

    auto log_energy = [&]() {
        double avg_tq = 0;
        double max_tq = 0;
        int cnt = 0;
        for (unsigned int i = 0; i < tet_qualities.size(); i++) {
            if (t_is_removed[i]) continue;
            if (tet_qualities[i].slim_energy > max_tq) max_tq = tet_qualities[i].slim_energy;
            avg_tq += tet_qualities[i].slim_energy;
            cnt++;
        }
        avg_tq /= cnt;
        logger().info("Energy: avg = {} | max = {}", avg_tq, max_tq);
    };

    if (ops[0]) {
        igl_timer.start();
        logger().info("edge splitting...");
        splitter.init();
        splitter.split();
        tmp_time = igl_timer.getElapsedTime();
        splitter.outputInfo(MeshRecord::OpType::OP_SPLIT, tmp_time, is_log);
        logger().info("edge splitting done!");
        log_energy();
        logger().info("time = {}s", tmp_time);
    }

    if (ops[1]) {
        igl_timer.start();
        logger().info("edge collapsing...");
        collapser.init();
        collapser.collapse();
        tmp_time = igl_timer.getElapsedTime();
        collapser.outputInfo(MeshRecord::OpType::OP_COLLAPSE, tmp_time, is_log);
        logger().info("edge collasing done!");
        log_energy();
        logger().info("time = {}s", tmp_time);
    }

    if (ops[2]) {
        igl_timer.start();
        logger().info("edge removing...");
        edge_remover.init();
        edge_remover.swap();
        tmp_time = igl_timer.getElapsedTime();
        edge_remover.outputInfo(MeshRecord::OpType::OP_SWAP, tmp_time, is_log);
        logger().info("edge removal done!");
        log_energy();
        logger().info("time = {}s", tmp_time);
    }

    if (ops[3]) {
        igl_timer.start();
        logger().info("vertex smoothing...");
        smoother.smooth();
        tmp_time = igl_timer.getElapsedTime();
        smoother.outputInfo(MeshRecord::OpType::OP_SMOOTH, tmp_time, is_log);
        logger().info("vertex smooth done!");
        log_energy();
        logger().info("time = {}s", tmp_time);
    }

    round();

    int cnt1 = 0;
    for (int i = 0; i < tet_vertices.size(); i++) {
        if (v_is_removed[i] || tet_vertices[i].is_locked || tet_vertices[i].is_rounded) continue;
        cnt1++;
    }
    return cnt0 - cnt1;
}

int MeshRefinement::doOperationLoops(
    EdgeSplitter& splitter,
    EdgeCollapser& collapser,
    EdgeRemover& edge_remover,
    VertexSmoother& smoother,
    int max_pass,
    const std::array<bool, 4>& ops)
{
    double avg_energy, max_energy;
    splitter.getAvgMaxEnergy(avg_energy, max_energy);

    int loop_cnt = 0;
    for (int i = 0; i < max_pass; i++) {
        doOperations(splitter, collapser, edge_remover, smoother, ops);
        loop_cnt++;

        double tmp_avg_energy, tmp_max_energy;
        splitter.getAvgMaxEnergy(tmp_avg_energy, tmp_max_energy);
        if (std::abs(tmp_avg_energy - avg_energy) < args.delta_energy_thres &&
            std::abs(tmp_max_energy - max_energy) < args.delta_energy_thres)
            break;
        avg_energy = tmp_avg_energy;
        max_energy = tmp_max_energy;
    }

    return loop_cnt;
}

void MeshRefinement::refine(
    const std::array<bool, 4>& ops,
    bool is_pre,
    bool is_post,
    int scalar_update)
{
    if (is_dealing_unrounded)
        min_adaptive_scale = state.eps / state.initial_edge_len * 0.5; // min to eps/2
    else
        //        min_adaptive_scale = state.eps_input / state.initial_edge_len; // state.eps_input / state.initial_edge_len * 0.5 is too small
        min_adaptive_scale =
            (state.bbox_diag / 1000) /
            state.initial_edge_len; // set min_edge_length to diag / 1000 would be better

    LocalOperations localOperation(
        tet_vertices,
        tets,
        is_surface_fs,
        v_is_removed,
        t_is_removed,
        tet_qualities,
        env_sf,
        env_b,
        args,
        state);
    EdgeSplitter splitter(
        localOperation,
        state.initial_edge_len * (4.0 / 3.0) * state.initial_edge_len * (4.0 / 3.0));
    EdgeCollapser collapser(
        localOperation,
        state.initial_edge_len * (4.0 / 5.0) * state.initial_edge_len * (4.0 / 5.0));
    EdgeRemover edge_remover(
        localOperation,
        state.initial_edge_len * (4.0 / 3.0) * state.initial_edge_len * (4.0 / 3.0));
    VertexSmoother smoother(localOperation);

    collapser.is_check_quality = true;

    if (is_pre) refine_pre(splitter, collapser, edge_remover, smoother);

    /// apply the local operations
    // if (is_dealing_unrounded) {
    //     for (int i = 0; i < tet_vertices.size(); i++) {
    //         if (v_is_removed[i] || tet_vertices[i].is_rounded) continue;
    //         smoother.outputOneRing(i, "");
    //     }
    // }

    double avg_energy0, max_energy0;
    localOperation.getAvgMaxEnergy(avg_energy0, max_energy0);
    double target_energy0 = 1e6;
    int update_buget = 2;
    int update_cnt = 0;
    int is_output = true;
    //    const double eps_s = 0.8;
    //    state.eps *= eps_s;
    //    state.eps_2 *= eps_s*eps_s;
    bool is_split = true;
    for (int pass = old_pass; pass < old_pass + args.max_num_passes; pass++) {
        if (is_dealing_unrounded && pass == old_pass) {
            updateScalarField(false, false, args.filter_energy_thres);
        }

        logger().info("//////////////// Pass {} ////////////////", pass);
        if (is_dealing_unrounded) collapser.is_limit_length = false;
        doOperations(
            splitter,
            collapser,
            edge_remover,
            smoother,
            std::array<bool, 4>({{is_split, ops[1], ops[2], ops[3]}}));
        update_cnt++;

        if (is_dealing_unrounded) {
            bool is_finished = true;
            for (int i = 0; i < tet_vertices.size(); i++) {
                if (v_is_removed[i]) continue;
                if (!tet_vertices[i].is_rounded) is_finished = false;
            }
            if (is_finished) {
                logger().debug("all vertices rounded!!");
                //                break;
            }
        }

        if (localOperation.getMaxEnergy() < args.filter_energy_thres) {
            logger().info("Max energy below threshold: {}", localOperation.getMaxEnergy());
            break;
        }

        // check and mark is_bad_element
        double avg_energy, max_energy;
        localOperation.getAvgMaxEnergy(avg_energy, max_energy);
        logger().info("Energy: avg = {} | max = {}", avg_energy, max_energy);
        if (pass > 0 && pass < old_pass + args.max_num_passes - 1 &&
            avg_energy0 - avg_energy < args.delta_energy_thres &&
            max_energy0 - max_energy < args.delta_energy_thres) {
            if (update_cnt == 1) {
                if (is_hit_min) {
                    update_buget--;
                } else {
                    continue;
                }
            }
            if (update_buget == 0) {
                if (state.sub_stage > 1 && state.sub_stage < args.stage) {
                    state.eps += state.eps_delta;
                    state.eps_2 = state.eps * state.eps;
                    state.sub_stage++;
                    update_buget = 2;
                } else {
                    logger().warn(">>>>>>>>>> update_budget = 0 -> break in pass {}", pass);
                    break;
                }
            }
            update_cnt = 0;

            // get target energy
            double target_energy = localOperation.getMaxEnergy() / 100;
            target_energy = std::min(target_energy, target_energy0 / 10);
            target_energy = std::max(target_energy, args.filter_energy_thres * 0.8);
            target_energy0 = target_energy;
            updateScalarField(false, false, target_energy);

            if (state.sub_stage == 1 && state.sub_stage < args.stage &&
                target_energy < args.filter_energy_thres) {
                state.eps += state.eps_delta;
                state.eps_2 = state.eps * state.eps;
                state.sub_stage++;
            }
        }
        avg_energy0 = avg_energy;
        max_energy0 = max_energy;
    }

    old_pass = old_pass + args.max_num_passes;

    if (is_post) {
        refine_post(splitter, collapser, edge_remover, smoother);
    }

    logger().info("Final max energy: {}", localOperation.getMaxEnergy());
}

void MeshRefinement::refine_pre(
    EdgeSplitter& splitter,
    EdgeCollapser& collapser,
    EdgeRemover& edge_remover,
    VertexSmoother& smoother)
{
    logger().info("////////////////// Pre-processing //////////////////");
    collapser.is_limit_length = false;
    doOperations(
        splitter,
        collapser,
        edge_remover,
        smoother,
        std::array<bool, 4>{{false, true, false, false}});
    collapser.is_limit_length = true;
}

void MeshRefinement::refine_post(
    EdgeSplitter& splitter,
    EdgeCollapser& collapser,
    EdgeRemover& edge_remover,
    VertexSmoother& smoother)
{
    logger().info("////////////////// Post-processing //////////////////");
    collapser.is_limit_length = true;
    for (int i = 0; i < tet_vertices.size(); i++) {
        tet_vertices[i].adaptive_scale = 1;
    }

    doOperations(
        splitter,
        collapser,
        edge_remover,
        smoother,
        std::array<bool, 4>{{false, true, false, false}});
}

int MeshRefinement::getInsideVertexSize()
{
    std::vector<bool> tmp_t_is_removed;
    markInOut(tmp_t_is_removed);
    std::unordered_set<int> inside_vs;
    for (int i = 0; i < tets.size(); i++) {
        if (tmp_t_is_removed[i]) continue;
        for (int j = 0; j < 4; j++) inside_vs.insert(tets[i][j]);
    }
    return inside_vs.size();
}

void MeshRefinement::markInOut(std::vector<bool>& tmp_t_is_removed)
{
    tmp_t_is_removed = t_is_removed;
    Eigen::MatrixXd C(std::count(tmp_t_is_removed.begin(), tmp_t_is_removed.end(), false), 3);
    int cnt = 0;
    for (int i = 0; i < tets.size(); i++) {
        if (tmp_t_is_removed[i]) continue;
        std::vector<Vector3d> vs;
        vs.reserve(4);
        for (int j = 0; j < 4; j++) {
            vs.push_back(tet_vertices[tets[i][j]].posf);
        }
        Vector3d p = (vs[0] + vs[1] + vs[2] + vs[3]) / 4;
        for (int j = 0; j < 3; j++) {
            C(cnt, j) = p[j];
        }
        cnt++;
    }

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    getSurface(V, F);
    Eigen::VectorXd W;
    logger().debug("winding number...");
    igl::winding_number(V, F, C, W);
    logger().debug("winding number done");

    cnt = 0;
    for (int i = 0; i < tets.size(); i++) {
        if (tmp_t_is_removed[i]) continue;
        tmp_t_is_removed[i] = !(W(cnt) > 0.5);
        cnt++;
    }
}

void MeshRefinement::updateScalarField(
    bool is_clean_up_unrounded,
    bool is_clean_up_local,
    double filter_energy,
    bool is_lock)
{
    // Whenever the mesh energy cannot be optimized too much (delta of avg and
    // max energy is < `delta_energy_thres`), we update the scalar field of the
    // target edge length. The update is performed as follows:
    // - Every vertex whose incident tets have a total energy below a given
    //   threshold is selected.
    // - For each selected vertex, place a ball around it (see code below for the
    //   radius).
    // - The scalar field is * `adaptive_scalar` (0.6 by default) at the center
    //   of the ball, left untouched at its boundary, and linearly interpolated
    //   in-between.

    igl_timer.start();
    logger().debug("marking adaptive scales...");
    double tmp_time = 0;

    double radius0 =
        state.initial_edge_len * 1.8; // increasing the radius would increase the #v in output
    if (is_hit_min) radius0 *= 2;
    if (is_clean_up_local) radius0 = state.initial_edge_len;
    if (is_clean_up_unrounded) radius0 *= 2;

    logger().debug("filter_energy_thres = {}", filter_energy);
    std::vector<double> adap_tmp(tet_vertices.size(), 1.5);
    double dynamic_adaptive_scale = args.adaptive_scalar;

    const int N = -int(std::log2(min_adaptive_scale) - 1);
    std::vector<std::vector<int>> v_ids(N, std::vector<int>());
    for (int i = 0; i < tet_vertices.size(); i++) {
        if (v_is_removed[i] || tet_vertices[i].is_locked) continue;

        if (is_clean_up_unrounded) {
            if (tet_vertices[i].is_rounded) continue;
        } else {
            bool is_refine = false;
            for (int t_id : tet_vertices[i].conn_tets) {
                if (tet_qualities[t_id].slim_energy > filter_energy) is_refine = true;
            }
            if (!is_refine) continue;
        }

        int n = -int(std::log2(tet_vertices[i].adaptive_scale) - 0.5);
        if (n >= N) n = N - 1;
        v_ids[n].push_back(i);
    }

    for (int n = 0; n < N; n++) {
        if (v_ids[n].size() == 0) continue;

        double radius = radius0 / std::pow(2, n);
        //        double radius = radius0 / 1.5;

        std::unordered_set<int> is_visited;
        std::queue<int> v_queue;

        std::vector<Vector3d> pts;
        pts.reserve(v_ids[n].size());
        for (int i = 0; i < v_ids[n].size(); i++) {
            // for (int j = 0; j < 3; j++) pts.push_back(tet_vertices[v_ids[n][i]].posf[j]);
            pts.emplace_back(tet_vertices[v_ids[n][i]].posf);

            v_queue.push(v_ids[n][i]);
            is_visited.insert(v_ids[n][i]);
            adap_tmp[v_ids[n][i]] = dynamic_adaptive_scale;
        }
        // construct the kdtree
        // GEO::NearestNeighborSearch_var nnsearch = GEO::NearestNeighborSearch::create(3, "BNN");
        // nnsearch->set_points(int(v_ids[n].size()), pts.data());
        wmtk::KNN knn(pts);

        while (!v_queue.empty()) {
            int v_id = v_queue.front();
            v_queue.pop();

            for (int t_id : tet_vertices[v_id].conn_tets) {
                for (int k = 0; k < 4; k++) {
                    if (is_visited.find(tets[t_id][k]) != is_visited.end()) continue;
                    double sq_dist;
                    // GEO::index_t _;
                    // const double p[3] = {
                    //     tet_vertices[tets[t_id][k]].posf[0],
                    //     tet_vertices[tets[t_id][k]].posf[1],
                    //     tet_vertices[tets[t_id][k]].posf[2]};
                    // nnsearch->get_nearest_neighbors(1, p, &_, &sq_dist);
                    uint32_t idx;
                    knn.nearest_neighbor(tet_vertices[tets[t_id][k]].posf, idx, sq_dist);
                    double dis = sqrt(sq_dist);

                    if (dis < radius && !tet_vertices[tets[t_id][k]].is_locked) {
                        v_queue.push(tets[t_id][k]);
                        double new_ss =
                            (dis / radius) * (1 - dynamic_adaptive_scale) + dynamic_adaptive_scale;
                        if (new_ss < adap_tmp[tets[t_id][k]]) adap_tmp[tets[t_id][k]] = new_ss;
                    }
                    is_visited.insert(tets[t_id][k]);
                }
            }
        }
    }

    // update scalars
    int cnt = 0;
    for (int i = 0; i < tet_vertices.size(); i++) {
        if (v_is_removed[i]) continue;
        if (is_clean_up_unrounded && is_lock && adap_tmp[i] > 1) {
            tet_vertices[i].is_locked = true;
            cnt++;
        }
        double new_scale = tet_vertices[i].adaptive_scale * adap_tmp[i];
        if (new_scale > 1)
            tet_vertices[i].adaptive_scale = 1;
        else if (new_scale < min_adaptive_scale) {
            if (!is_clean_up_unrounded) is_hit_min = true;
            tet_vertices[i].adaptive_scale = min_adaptive_scale;
        } else
            tet_vertices[i].adaptive_scale = new_scale;
    }
    if (is_clean_up_unrounded && is_lock) logger().debug("{} vertices locked", cnt);

    logger().debug("marked!");
    tmp_time = igl_timer.getElapsedTime();
    logger().debug("time = {}s", tmp_time);
    //    outputMidResult(true);
}

void MeshRefinement::getSurface(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    std::vector<std::array<int, 3>> fs;
    std::vector<int> vs;
    for (int i = 0; i < tets.size(); i++) {
        if (t_is_removed[i]) continue;
        for (int j = 0; j < 4; j++) {
            if (is_surface_fs[i][j] != state.NOT_SURFACE && is_surface_fs[i][j] > 0) { // outside
                std::array<int, 3> v_ids = {
                    {tets[i][(j + 1) % 4], tets[i][(j + 2) % 4], tets[i][(j + 3) % 4]}};

                Vector3r n = ((tet_vertices[v_ids[1]].pos) - tet_vertices[v_ids[0]].pos)
                                 .cross((tet_vertices[v_ids[2]].pos) - tet_vertices[v_ids[0]].pos);
                Vector3r d = (tet_vertices[v_ids[3]].pos) - tet_vertices[v_ids[0]].pos;
                auto res = n.dot(d);

                if (res <= 0) {
                    int tmp = v_ids[0];
                    v_ids[0] = v_ids[2];
                    v_ids[2] = tmp;
                }
                for (int k = 0; k < is_surface_fs[i][j]; k++) fs.push_back(v_ids);
                for (int k = 0; k < 3; k++) vs.push_back(v_ids[k]);
            }
        }
    }
    std::sort(vs.begin(), vs.end());
    vs.erase(std::unique(vs.begin(), vs.end()), vs.end());

    V.resize(vs.size(), 3);
    std::map<int, int> map_ids;
    for (int i = 0; i < vs.size(); i++) {
        map_ids[vs[i]] = i;
        for (int j = 0; j < 3; j++) V(i, j) = tet_vertices[vs[i]].posf[j];
    }

    F.resize(fs.size(), 3);
    for (int i = 0; i < fs.size(); i++) {
        for (int j = 0; j < 3; j++) F(i, j) = map_ids[fs[i][j]];
    }
}

void MeshRefinement::getTrackedSurface(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    std::vector<std::array<int, 6>> fs;
    std::vector<int> vs;
    for (int i = 0; i < tets.size(); i++) {
        if (t_is_removed[i]) continue;
        for (int j = 0; j < 4; j++) {
            if (is_surface_fs[i][j] != state.NOT_SURFACE && is_surface_fs[i][j] >= 0) { // outside
                std::array<int, 3> v_ids = {
                    {tets[i][(j + 1) % 4], tets[i][(j + 2) % 4], tets[i][(j + 3) % 4]}};

                Vector3r n = ((tet_vertices[v_ids[1]].pos) - tet_vertices[v_ids[0]].pos)
                                 .cross((tet_vertices[v_ids[2]].pos) - tet_vertices[v_ids[0]].pos);
                Vector3r d = (tet_vertices[v_ids[3]].pos) - tet_vertices[v_ids[0]].pos;
                auto res = n.dot(d);

                if (res <= 0) {
                    int tmp = v_ids[0];
                    v_ids[0] = v_ids[2];
                    v_ids[2] = tmp;
                }
                std::array<int, 3> v_ids1 = v_ids;
                std::sort(v_ids1.begin(), v_ids1.end());
                fs.push_back(std::array<int, 6>(
                    {{v_ids1[0], v_ids1[1], v_ids1[2], v_ids[0], v_ids[1], v_ids[2]}}));
                for (int k = 0; k < 3; k++) vs.push_back(v_ids[k]);
            }
        }
    }
    std::sort(vs.begin(), vs.end());
    vs.erase(std::unique(vs.begin(), vs.end()), vs.end());

    V.resize(vs.size(), 3);
    std::unordered_map<int, int> map_ids;
    for (int i = 0; i < vs.size(); i++) {
        map_ids[vs[i]] = i;
        for (int j = 0; j < 3; j++) V(i, j) = tet_vertices[vs[i]].posf[j];
    }

    F.resize(fs.size(), 3);
    for (int i = 0; i < fs.size(); i++)
        for (int j = 0; j < 3; j++) {
            F(i, j) = map_ids[fs[i][j + 3]];
        }

    return;

    std::sort(fs.begin(), fs.end());
    int nf = 0;
    for (int i = 0; i < fs.size(); i++) {
        if (i > 0 && fs[i][0] == fs[i - 1][0] && fs[i][1] == fs[i - 1][1] &&
            fs[i][2] == fs[i - 1][2])
            continue;
        nf++;
    }

    F.resize(nf, 3);
    int cnt = 0;
    for (int i = 0; i < fs.size(); i++) {
        if (i > 0 && fs[i][0] == fs[i - 1][0] && fs[i][1] == fs[i - 1][1] &&
            fs[i][2] == fs[i - 1][2])
            continue;
        for (int j = 0; j < 3; j++) {
            F(cnt, j) = map_ids[fs[i][j + 3]];
            cnt++;
        }
    }
}

void MeshRefinement::getTrackedSurface_continuous(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    std::vector<std::array<int, 6>> fs;

    for (int i = 0; i < tets.size(); i++) {
        if (t_is_removed[i]) {
            continue;
        }
        for (int j = 0; j < 4; j++) {
            if (is_surface_fs[i][j] != state.NOT_SURFACE && is_surface_fs[i][j] >= 0) { // outside
                std::array<int, 3> v_ids = {
                    {tets[i][(j + 1) % 4], tets[i][(j + 2) % 4], tets[i][(j + 3) % 4]}};

                Vector3r n = ((tet_vertices[v_ids[1]].pos) - tet_vertices[v_ids[0]].pos)
                                 .cross((tet_vertices[v_ids[2]].pos) - tet_vertices[v_ids[0]].pos);
                Vector3r d = (tet_vertices[v_ids[3]].pos) - tet_vertices[v_ids[0]].pos;
                auto res = n.dot(d);

                if (res <= 0) {
                    int tmp = v_ids[0];
                    v_ids[0] = v_ids[2];
                    v_ids[2] = tmp;
                }
                std::array<int, 3> v_ids1 = v_ids;
                std::sort(v_ids1.begin(), v_ids1.end());
                fs.push_back(std::array<int, 6>(
                    {{v_ids1[0], v_ids1[1], v_ids1[2], v_ids[0], v_ids[1], v_ids[2]}}));
            }
        }
    }

    V.resize(tet_vertices.size(), 3);
    for (int i = 0; i < tet_vertices.size(); i++) {
        V.row(i) = tet_vertices[i].posf;
    }

    F.resize(fs.size(), 3);
    for (int i = 0; i < fs.size(); i++) {
        for (int j = 0; j < 3; j++) {
            F(i, j) = fs[i][j + 3];
        }
    }
}

} // namespace wmtk::components::tetwild::orig
