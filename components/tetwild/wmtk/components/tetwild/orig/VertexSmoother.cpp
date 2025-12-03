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
        if (!NewtonsMethod(t_ids, new_tets, v_id, pf)) {
            return false;
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

        igl_timer.start();
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
        breakdown_timing[id_round] += igl_timer.getElapsedTime();

        if (!is_valid) {
            continue;
        } else {
            Vector3d pf;
            if (!NewtonsMethod(t_ids, new_tets, v_id, pf)) {
                continue;
            }
            igl_timer.start();
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
            breakdown_timing[id_round] += igl_timer.getElapsedTime();
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
    igl_timer.start();
    calTetQualities(new_tets, tet_qs);
    breakdown_timing[id_value_e] += igl_timer.getElapsedTime();
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
            if (!NewtonsMethod(old_t_ids, new_tets, v_id, pf_out)) {
                continue;
            }
            p_out = Vector3r(pf_out[0], pf_out[1], pf_out[2]);
        }

        ///find one-ring surface faces
        igl_timer.start();
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
        breakdown_timing[id_project] += igl_timer.getElapsedTime();

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
        if (!new_tq.isBetterThan(old_tq, state)) {
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

        igl_timer.start();

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
        breakdown_timing[id_aabb] += igl_timer.getElapsedTime();
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
        double new_energy = old_energy;

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
        s_energy += AMIPS_energy(t);
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

} // namespace wmtk::components::tetwild::orig
