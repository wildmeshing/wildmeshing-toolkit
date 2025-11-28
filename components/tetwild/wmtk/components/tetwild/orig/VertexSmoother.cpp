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

#include "VertexSmoother.h"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/Logger.hpp>
#include "Common.h"

namespace wmtk::components::tetwild::orig {

void VertexSmoother::smooth()
{
    tets_tss = std::vector<int>(tets.size(), 1);
    tet_vertices_tss = std::vector<int>(tet_vertices.size(), 0);
    ts = 1;

    igl::Timer tmp_timer0;
    int max_pass = 1;
    double v_cnt = std::count(v_is_removed.begin(), v_is_removed.end(), false);
    for (int i = 0; i < max_pass; i++) {
        double suc_in = 0;
        double suc_surface = 0;
        smoothSingle();
        suc_in = suc_counter;
        if (state.eps >= 0) {
            smoothSurface();
            suc_surface = suc_counter;
        }
        logger().debug("{}", (suc_in + suc_surface) / v_cnt);
        if (suc_in + suc_surface < v_cnt * 0.1) {
            logger().debug("{}", i);
            break;
        }
    }
    for (int i = 0; i < breakdown_timing.size(); i++) {
        logger().debug("{}: {}s", breakdown_name[i], breakdown_timing[i]);
        breakdown_timing[i] = 0; // reset
    }
}

bool VertexSmoother::smoothSingleVertex(int v_id, bool is_cal_energy)
{
    std::vector<std::array<int, 4>> new_tets;
    std::vector<int> t_ids;
    for (int t_id : tet_vertices[v_id].conn_tets) {
        new_tets.push_back(tets[t_id]);
        t_ids.push_back(t_id);
    }

    ///try to round the vertex
    if (!tet_vertices[v_id].is_rounded) {
        Vector3r old_p = tet_vertices[v_id].pos;
        tet_vertices[v_id].pos = to_rational(tet_vertices[v_id].posf);
        if (isFlip(new_tets))
            tet_vertices[v_id].pos = old_p;
        else
            tet_vertices[v_id].is_rounded = true;
    }

    ///check if should use exact smoothing
    bool is_valid = true;
    for (auto it = tet_vertices[v_id].conn_tets.begin(); it != tet_vertices[v_id].conn_tets.end();
         it++) {
        auto res = igl::predicates::orient3d(
            tet_vertices[tets[*it][0]].posf,
            tet_vertices[tets[*it][1]].posf,
            tet_vertices[tets[*it][2]].posf,
            tet_vertices[tets[*it][3]].posf);

        if (res != igl::predicates::Orientation::NEGATIVE) {
            is_valid = false;
            break;
        }
    }
    if (!is_valid) {
        return false;
    } else {
        Vector3d pf;
        if (energy_type == state.ENERGY_AMIPS) {
            if (!NewtonsMethod(t_ids, new_tets, v_id, pf)) return false;
        }

        // assign new coordinate and try to round it
        Vector3r old_p = tet_vertices[v_id].pos;
        Vector3d old_pf = tet_vertices[v_id].posf;
        bool old_is_rounded = tet_vertices[v_id].is_rounded;
        Vector3r p(pf[0], pf[1], pf[2]);
        tet_vertices[v_id].pos = p;
        tet_vertices[v_id].posf = pf;
        tet_vertices[v_id].is_rounded = true;
        if (isFlip(new_tets)) { // TODO: why it happens?
            logger().debug("flip in the end");
            tet_vertices[v_id].pos = old_p;
            tet_vertices[v_id].posf = old_pf;
            tet_vertices[v_id].is_rounded = old_is_rounded;
        }
    }

    if (is_cal_energy) {
        std::vector<TetQuality> tet_qs;
        calTetQualities(new_tets, tet_qs);
        int cnt = 0;
        for (int t_id : tet_vertices[v_id].conn_tets) {
            tet_qualities[t_id] = tet_qs[cnt++];
        }
    }

    return true;
}

void VertexSmoother::smoothSingle()
{
    double old_ts = ts;
    counter = 0;
    suc_counter = 0;
    for (int v_id = 0; v_id < tet_vertices.size(); v_id++) {
        if (v_is_removed[v_id]) continue;
        if (tet_vertices[v_id].is_on_bbox) continue;
        if (state.eps != state.EPSILON_INFINITE && tet_vertices[v_id].is_on_surface) continue;

        if (tet_vertices[v_id].is_locked) continue;

        ///check if its one-ring is changed
        //        bool is_changed=false;
        //        for(auto
        //        it=tet_vertices[v_id].conn_tets.begin();it!=tet_vertices[v_id].conn_tets.end();it++){
        //            if(tets_tss[*it]>tet_vertices_tss[v_id]){
        //                is_changed=true;
        //                break;
        //            }
        //        }
        //        if(!is_changed)
        //            continue;

        counter++;

#if TIMING_BREAKDOWN
        igl_timer.start();
#endif
        std::vector<std::array<int, 4>> new_tets;
        std::vector<int> t_ids;
        for (auto it = tet_vertices[v_id].conn_tets.begin();
             it != tet_vertices[v_id].conn_tets.end();
             it++) {
            new_tets.push_back(tets[*it]);
            t_ids.push_back(*it);
        }

        ///try to round the vertex
        if (!tet_vertices[v_id].is_rounded) {
            Vector3r old_p = tet_vertices[v_id].pos;
            tet_vertices[v_id].pos = to_rational(tet_vertices[v_id].posf);
            if (isFlip(new_tets))
                tet_vertices[v_id].pos = old_p;
            else
                tet_vertices[v_id].is_rounded = true;
        }

        ///check if should use exact smoothing
        bool is_valid = true;
        for (auto it = tet_vertices[v_id].conn_tets.begin();
             it != tet_vertices[v_id].conn_tets.end();
             it++) {
            auto res = igl::predicates::orient3d(
                tet_vertices[tets[*it][0]].posf,
                tet_vertices[tets[*it][1]].posf,
                tet_vertices[tets[*it][2]].posf,
                tet_vertices[tets[*it][3]].posf);

            if (res != igl::predicates::Orientation::NEGATIVE) {
                is_valid = false;
                break;
            }
        }
#if TIMING_BREAKDOWN
        breakdown_timing[id_round] += igl_timer.getElapsedTime();
#endif

        if (!is_valid) {
            continue;
        } else {
            Vector3d pf;
            if (energy_type == state.ENERGY_AMIPS) {
                if (!NewtonsMethod(t_ids, new_tets, v_id, pf)) continue;
            }
#if TIMING_BREAKDOWN
            igl_timer.start();
#endif
            // assign new coordinate and try to round it
            Vector3r old_p = tet_vertices[v_id].pos;
            Vector3d old_pf = tet_vertices[v_id].posf;
            bool old_is_rounded = tet_vertices[v_id].is_rounded;
            Vector3r p(pf[0], pf[1], pf[2]);
            tet_vertices[v_id].pos = p;
            tet_vertices[v_id].posf = pf;
            tet_vertices[v_id].is_rounded = true;
            if (isFlip(new_tets)) { // TODO: why it happens?
                logger().debug("flip in the end");
                tet_vertices[v_id].pos = old_p;
                tet_vertices[v_id].posf = old_pf;
                tet_vertices[v_id].is_rounded = old_is_rounded;
            }
#if TIMING_BREAKDOWN
            breakdown_timing[id_round] += igl_timer.getElapsedTime();
#endif
        }

        ///update timestamps
        ts++;
        for (auto it = tet_vertices[v_id].conn_tets.begin();
             it != tet_vertices[v_id].conn_tets.end();
             it++)
            tets_tss[*it] = ts;
        tet_vertices_tss[v_id] = ts;

        suc_counter++;
    }

    // calculate the quality for all tets
    std::vector<std::array<int, 4>> new_tets; // todo: can be improve
    new_tets.reserve(std::count(t_is_removed.begin(), t_is_removed.end(), false));
    for (int i = 0; i < tets.size(); i++) {
        if (t_is_removed[i]) continue;
        //        if(tets_tss[i]<=old_ts)
        //            continue;
        new_tets.push_back(tets[i]);
    }
    std::vector<TetQuality> tet_qs;
#if TIMING_BREAKDOWN
    igl_timer.start();
#endif
    calTetQualities(new_tets, tet_qs);
#if TIMING_BREAKDOWN
    breakdown_timing[id_value_e] += igl_timer.getElapsedTime();
#endif
    int cnt = 0;
    for (int i = 0; i < tets.size(); i++) {
        if (t_is_removed[i]) continue;
        //        if(tets_tss[i]<=old_ts)
        //            continue;
        tet_qualities[i] = tet_qs[cnt++];
    }
}

void VertexSmoother::smoothSurface()
{ // smoothing surface using two methods
    //    suc_counter = 0;
    //    counter = 0;
    int sf_suc_counter = 0;
    int sf_counter = 0;

    for (int v_id = 0; v_id < tet_vertices.size(); v_id++) {
        if (v_is_removed[v_id]) continue;
        if (!tet_vertices[v_id].is_on_surface) continue;

        if (tet_vertices[v_id].is_locked) continue;

        if (isIsolated(v_id)) {
            tet_vertices[v_id].is_on_surface = false;
            tet_vertices[v_id].is_on_boundary = false;
            tet_vertices[v_id].on_fixed_vertex = -1;
            tet_vertices[v_id].on_face.clear();
            tet_vertices[v_id].on_edge.clear();
            continue;
        }
        if (!isBoundaryPoint(v_id)) tet_vertices[v_id].is_on_boundary = false;

        counter++;
        sf_counter++;

        std::vector<std::array<int, 4>> new_tets;
        std::vector<int> old_t_ids;
        for (auto it = tet_vertices[v_id].conn_tets.begin();
             it != tet_vertices[v_id].conn_tets.end();
             it++) {
            new_tets.push_back(tets[*it]);
            old_t_ids.push_back(*it);
        }

        if (!tet_vertices[v_id].is_rounded) {
            Vector3r old_p = tet_vertices[v_id].pos;
            tet_vertices[v_id].pos = to_rational(tet_vertices[v_id].posf);
            if (isFlip(new_tets))
                tet_vertices[v_id].pos = old_p;
            else
                tet_vertices[v_id].is_rounded = true;
        }

        bool is_valid = true;
        for (auto it = tet_vertices[v_id].conn_tets.begin();
             it != tet_vertices[v_id].conn_tets.end();
             it++) {
            auto res = igl::predicates::orient3d(
                tet_vertices[tets[*it][0]].posf,
                tet_vertices[tets[*it][1]].posf,
                tet_vertices[tets[*it][2]].posf,
                tet_vertices[tets[*it][3]].posf);

            if (res != igl::predicates::Orientation::NEGATIVE) {
                is_valid = false;
                break;
            }
        }

        Vector3r p_out;
        Vector3d pf_out;
        if (!is_valid) {
            continue;
        } else {
            if (energy_type == state.ENERGY_AMIPS) {
                if (!NewtonsMethod(old_t_ids, new_tets, v_id, pf_out)) continue;
            }
            p_out = Vector3r(pf_out[0], pf_out[1], pf_out[2]);
        }

        ///find one-ring surface faces
#if TIMING_BREAKDOWN
        igl_timer.start();
#endif
        std::vector<std::array<int, 3>> tri_ids;
        for (auto it = tet_vertices[v_id].conn_tets.begin();
             it != tet_vertices[v_id].conn_tets.end();
             it++) {
            for (int j = 0; j < 4; j++) {
                if (tets[*it][j] != v_id && is_surface_fs[*it][j] != state.NOT_SURFACE) {
                    std::array<int, 3> tri = {
                        {tets[*it][(j + 1) % 4], tets[*it][(j + 2) % 4], tets[*it][(j + 3) % 4]}};
                    std::sort(tri.begin(), tri.end());
                    tri_ids.push_back(tri);
                }
            }
        }
        std::sort(tri_ids.begin(), tri_ids.end());
        tri_ids.erase(std::unique(tri_ids.begin(), tri_ids.end()), tri_ids.end());

        Vector3d pf;
        Vector3r p;
        if (state.use_onering_projection) { // we have to use exact construction here. Or the
                                            // projecting points may be not exactly on the plane.
            log_and_throw_error("removed code that appeared deprecated");
        } else {
            Vector3d nearest_pf;
            if (tet_vertices[v_id].is_on_boundary)
                geo_b_tree.nearest_point(pf_out, nearest_pf);
            else
                geo_sf_tree.nearest_point(pf_out, nearest_pf);
            pf = nearest_pf;
            p = to_rational(nearest_pf);
        }
#if TIMING_BREAKDOWN
        breakdown_timing[id_project] += igl_timer.getElapsedTime();
#endif

        Vector3r old_p = tet_vertices[v_id].pos;
        Vector3d old_pf = tet_vertices[v_id].posf;
        std::vector<TetQuality> tet_qs;
        bool is_found = false;

        tet_vertices[v_id].posf = pf;
        tet_vertices[v_id].pos = p;
        if (isFlip(new_tets)) {
            tet_vertices[v_id].pos = old_p;
            tet_vertices[v_id].posf = old_pf;
            continue;
        }
        TetQuality old_tq, new_tq;
        getCheckQuality(old_t_ids, old_tq);
        calTetQualities(new_tets, tet_qs);
        getCheckQuality(tet_qs, new_tq);
        if (!new_tq.isBetterThan(old_tq, energy_type, state)) {
            tet_vertices[v_id].pos = old_p;
            tet_vertices[v_id].posf = old_pf;
            continue;
        }
        is_found = true;

        if (!is_found) {
            tet_vertices[v_id].pos = old_p;
            tet_vertices[v_id].posf = old_pf;
            continue;
        }

#if TIMING_BREAKDOWN
        igl_timer.start();
#endif

        ///check if tris outside the envelop
        std::vector<std::array<Vector3d, 3>> trisf;
        for (int i = 0; i < tri_ids.size(); i++) {
            auto jt = std::find(tri_ids[i].begin(), tri_ids[i].end(), v_id);
            int k = jt - tri_ids[i].begin();
            std::array<Vector3d, 3> tri{
                to_double(p),
                tet_vertices[tri_ids[i][(k + 1) % 3]].posf,
                tet_vertices[tri_ids[i][(k + 2) % 3]].posf};
            trisf.push_back(tri);
        }

        is_valid = true;
        for (int i = 0; i < trisf.size(); i++) {
            if (isFaceOutEnvelop(trisf[i])) {
                is_valid = false;
                break;
            }
        }
#if TIMING_BREAKDOWN
        breakdown_timing[id_aabb] += igl_timer.getElapsedTime();
#endif
        if (!is_valid) {
            tet_vertices[v_id].pos = old_p;
            tet_vertices[v_id].posf = old_pf;
            continue;
        }

        ///real update
        ///update timestamps
        ts++;
        for (auto it = tet_vertices[v_id].conn_tets.begin();
             it != tet_vertices[v_id].conn_tets.end();
             it++)
            tets_tss[*it] = ts;
        tet_vertices_tss[v_id] = ts;

        if (!tet_vertices[v_id].is_rounded) {
            tet_vertices[v_id].pos = Vector3r(pf[0], pf[1], pf[2]);
            if (isFlip(new_tets)) {
                tet_vertices[v_id].pos = old_p;
                tet_vertices[v_id].is_rounded = false;
            } else
                tet_vertices[v_id].is_rounded = true;
        }
        for (int i = 0; i < old_t_ids.size(); i++) tet_qualities[old_t_ids[i]] = tet_qs[i];

        suc_counter++;
        sf_suc_counter++;
        if (sf_suc_counter % 1000 == 0) logger().debug("1000 accepted!");
    }
    logger().debug("Totally {}({}) vertices on surface are smoothed.", sf_suc_counter, sf_counter);
}

bool VertexSmoother::NewtonsMethod(
    const std::vector<int>& t_ids,
    const std::vector<std::array<int, 4>>& new_tets,
    int v_id,
    Vector3d& p)
{
    //    bool is_moved=true;
    bool is_moved = false;
    const int MAX_STEP = 15;
    const int MAX_IT = 20;
    Vector3d pf0 = tet_vertices[v_id].posf;
    Vector3r p0 = tet_vertices[v_id].pos;

    double old_energy = 0;
    Eigen::Vector3d J;
    Eigen::Matrix3d H;
    Eigen::Vector3d X0;
    for (int step = 0; step < MAX_STEP; step++) {
        if (NewtonsUpdate(t_ids, v_id, old_energy, J, H, X0) == false) break;
        Vector3d old_pf = tet_vertices[v_id].posf;
        Vector3r old_p = tet_vertices[v_id].pos;
        double a = 1;
        bool step_taken = false;
        double new_energy;

        for (int it = 0; it < MAX_IT; it++) {
            // solve linear system
            // check flip
            // check energy
            igl_timer.start();
            Eigen::Vector3d X = H.colPivHouseholderQr().solve(H * X0 - a * J);
            breakdown_timing[id_solve] += igl_timer.getElapsedTime();
            if (!X.allFinite()) {
                a /= 2.0;
                continue;
            }

            tet_vertices[v_id].posf = Vector3d(X(0), X(1), X(2));
            tet_vertices[v_id].pos = Vector3r(X(0), X(1), X(2));
            //            tet_vertices[v_id].is_rounded=true;//need to remember old value?

            // check flipping
            if (isFlip(new_tets)) {
                tet_vertices[v_id].posf = old_pf;
                tet_vertices[v_id].pos = old_p;
                a /= 2.0;
                continue;
            }

            // check quality
            igl_timer.start();
            new_energy = getNewEnergy(t_ids);
            breakdown_timing[id_value_e] += igl_timer.getElapsedTime();
            if (new_energy >= old_energy || std::isinf(new_energy) || std::isnan(new_energy)) {
                tet_vertices[v_id].posf = old_pf;
                tet_vertices[v_id].pos = old_p;
                a /= 2.0;
                continue;
            }

            step_taken = true;
            break;
        }
        if (std::abs(new_energy - old_energy) < 1e-5) step_taken = false;

        if (!step_taken) {
            if (step == 0)
                is_moved = false;
            else
                is_moved = true;
            break;
        } else
            is_moved = true;
    }
    p = tet_vertices[v_id].posf;
    tet_vertices[v_id].posf = pf0;
    tet_vertices[v_id].pos = p0;

    return is_moved;
}

double VertexSmoother::getNewEnergy(const std::vector<int>& t_ids)
{
    double s_energy = 0;

    for (int i = 0; i < t_ids.size(); i++) {
        std::array<double, 12> t;
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 3; k++) {
                t[j * 3 + k] = tet_vertices[tets[t_ids[i]][j]].posf[k];
            }
        }
        if (energy_type == state.ENERGY_AMIPS) {
            s_energy += AMIPS_energy(t);
        }
    }

    if (std::isinf(s_energy) || std::isnan(s_energy) || s_energy <= 0 ||
        s_energy > state.MAX_ENERGY) {
        logger().debug("new E inf");
        s_energy = state.MAX_ENERGY;
    }

    return s_energy;
}

bool VertexSmoother::NewtonsUpdate(
    const std::vector<int>& t_ids,
    int v_id,
    double& energy,
    Eigen::Vector3d& J,
    Eigen::Matrix3d& H,
    Eigen::Vector3d& X0)
{
    energy = 0;
    for (int i = 0; i < 3; i++) {
        J(i) = 0;
        for (int j = 0; j < 3; j++) {
            H(i, j) = 0;
        }
        X0(i) = tet_vertices[v_id].posf[i];
    }

    for (int i = 0; i < t_ids.size(); i++) {
        std::array<double, 12> t;
        int start = 0;
        for (int j = 0; j < 4; j++) {
            if (tets[t_ids[i]][j] == v_id) {
                start = j;
                break;
            }
        }
        for (int j = 0; j < 4; j++) {
            for (int k = 0; k < 3; k++) {
                t[j * 3 + k] = tet_vertices[tets[t_ids[i]][(start + j) % 4]].posf[k];
            }
        }
        igl_timer.start();
        energy += AMIPS_energy(t);
        breakdown_timing[id_value_e] += igl_timer.getElapsedTime();

        Vector3d J_1;
        Eigen::Matrix3d H_1;
        igl_timer.start();
        AMIPS_jacobian(t, J_1);
        breakdown_timing[id_value_j] += igl_timer.getElapsedTime();
        igl_timer.start();
        AMIPS_hessian(t, H_1);
        breakdown_timing[id_value_h] += igl_timer.getElapsedTime();

        for (int j = 0; j < 3; j++) {
            J(j) += J_1[j];
            H(j, 0) += H_1(j, 0);
            H(j, 1) += H_1(j, 1);
            H(j, 2) += H_1(j, 2);
        }
    }

    if (std::isinf(energy)) {
        logger().debug("{} E inf", v_id);
        energy = state.MAX_ENERGY;
    }
    if (std::isnan(energy)) {
        logger().debug("{} E nan", v_id);
        return false;
    }
    if (energy <= 0) {
        logger().debug("{} E < 0", v_id);
        return false;
    }
    if (!J.allFinite()) {
        logger().debug("{} J inf/nan", v_id);
        return false;
    }
    if (!H.allFinite()) {
        logger().debug("{} H inf/nan", v_id);
        return false;
    }

    return true;
}

int VertexSmoother::laplacianBoundary(
    const std::vector<int>& b_v_ids,
    const std::vector<bool>& tmp_is_on_surface,
    const std::vector<bool>& tmp_t_is_removed)
{
    int cnt_suc = 0;
    double max_slim_evergy = 0;
    for (unsigned int i = 0; i < tet_qualities.size(); i++) {
        if (tmp_t_is_removed[i]) continue;
        if (tet_qualities[i].slim_energy > max_slim_evergy)
            max_slim_evergy = tet_qualities[i].slim_energy;
    }

    for (int v_id : b_v_ids) {
        // do laplacian on v_id
        std::vector<std::array<int, 4>> new_tets;
        std::unordered_set<int> n_v_ids;
        //        std::unordered_set<int> n_v_ids2;
        std::unordered_set<int> tmp_n_sf_v_ids;
        std::unordered_set<int> n_sf_v_ids;
        for (int t_id : tet_vertices[v_id].conn_tets) {
            for (int j = 0; j < 4; j++) {
                if (tmp_is_on_surface[tets[t_id][j]]) {
                    tmp_n_sf_v_ids.insert(tets[t_id][j]);
                    continue;
                }
                if (!tet_vertices[tets[t_id][j]].is_on_surface) n_v_ids.insert(tets[t_id][j]);
            }
            new_tets.push_back(tets[t_id]);
        }
        for (int n_sf_v_id : tmp_n_sf_v_ids) {
            std::vector<int> t_ids;
            setIntersection(tet_vertices[v_id].conn_tets, tet_vertices[n_sf_v_id].conn_tets, t_ids);
            bool has_removed = false;
            bool has_unremoved = false;
            for (int t_id : t_ids) {
                if (tmp_t_is_removed[t_id]) has_removed = true;
                if (!tmp_t_is_removed[t_id]) has_unremoved = true;
            }
            if (has_removed && has_unremoved) n_sf_v_ids.insert(n_sf_v_id);
        }
        //        for(int n_v_id:n_v_ids){
        //            for(int t_id:tet_vertices[n_v_id].conn_tets){
        //                for(int j=0;j<4;j++)
        //                    if(!tmp_is_on_surface[tets[t_id][j]] &&
        //                    !tet_vertices[tets[t_id][j]].is_on_surface)
        //                        n_v_ids2.insert(tets[t_id][j]);
        //            }
        //        }
        std::array<double, 3> vec = {{0, 0, 0}};
        for (int n_sf_v_id : n_sf_v_ids) {
            for (int j = 0; j < 3; j++) vec[j] += tet_vertices[n_sf_v_id].posf[j];
        }
        for (int j = 0; j < 3; j++) {
            vec[j] = (vec[j] / n_sf_v_ids.size()) - tet_vertices[v_id].posf[j];
        }

        // do bisection and check flipping
        Vector3r old_p = tet_vertices[v_id].pos;
        Vector3d old_pf = tet_vertices[v_id].posf;
        double a = 1;
        bool is_suc = false;
        while (true) {
            // give stop condition
            bool is_stop = true;
            for (int j = 0; j < 3; j++)
                if (vec[j] * a > state.eps) is_stop = false;
            if (is_stop) break;
            tet_vertices[v_id].pos =
                Vector3r(old_pf[0] + vec[0] * a, old_pf[1] + vec[1] * a, old_pf[2] + vec[2] * a);
            tet_vertices[v_id].posf =
                Vector3d(old_pf[0] + vec[0] * a, old_pf[1] + vec[1] * a, old_pf[2] + vec[2] * a);
            if (isFlip(new_tets)) {
                a /= 2;
                continue;
            }
            // check quality
            std::vector<TetQuality> tet_qs;
            calTetQualities(new_tets, tet_qs);
            bool is_valid = true;
            for (int i = 0; i < tet_qs.size(); i++) {
                if (tet_qs[i].slim_energy > max_slim_evergy) is_valid = false;
            }
            if (!is_valid) {
                a /= 2;
                continue;
            }

            int cnt = 0;
            for (int t_id : tet_vertices[v_id].conn_tets) {
                tet_qualities[t_id] = tet_qs[cnt++];
            }

            is_suc = true;
            cnt_suc++;
            break;
        }
        if (!is_suc) {
            tet_vertices[v_id].pos = old_p;
            tet_vertices[v_id].posf = old_pf;
            continue;
        }

        std::vector<TetQuality> tet_qs;
        calTetQualities(new_tets, tet_qs);
        int cnt = 0;
        for (int t_id : tet_vertices[v_id].conn_tets) {
            tet_qualities[t_id] = tet_qs[cnt++];
        }

        // do normal smoothing on neighbor vertices
        //        logger().debug("n_v_ids.size = {}", n_v_ids.size());
        //        logger().debug("n_v_ids2.size = {}", n_v_ids2.size());
        for (int n_v_id : n_v_ids) {
            smoothSingleVertex(n_v_id, true);
        }
        //        for(int n_v_id:n_v_ids2){
        //            smoothSingleVertex(n_v_id, true);
        //        }
        //        for(int n_v_id:n_v_ids){
        //            smoothSingleVertex(n_v_id, true);
        //        }
    }

    logger().debug("suc.size = {}", cnt_suc);
    return cnt_suc;
}

} // namespace wmtk::components::tetwild::orig
