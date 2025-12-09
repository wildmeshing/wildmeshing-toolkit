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

#include "EdgeCollapser.h"

#include <igl/Timer.h>
#include <wmtk/utils/Logger.hpp>

#include "Common.h"

namespace wmtk::components::tetwild::orig {

void EdgeCollapser::init()
{
    energy_time = 0;

    ////cal dir_edge
    // find all edges
    // check if collapsable 1
    // if yes, insert it into queue
    const unsigned int tets_size = tets.size();
    std::vector<std::array<int, 2>> edges;
    edges.reserve(tets_size * 6);
    for (unsigned int i = 0; i < tets.size(); i++) {
        if (t_is_removed[i]) continue;
        for (int j = 0; j < 3; j++) {
            std::array<int, 2> e = {{tets[i][0], tets[i][j + 1]}};
            if (e[0] > e[1]) e = {{e[1], e[0]}};
            if (!isLocked_ui(e)) edges.push_back(e);
            e = {{tets[i][j + 1], tets[i][(j + 1) % 3 + 1]}};
            if (e[0] > e[1]) e = {{e[1], e[0]}};
            if (!isLocked_ui(e)) edges.push_back(e);
        }
    }
    std::sort(edges.begin(), edges.end());
    edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
    logger().info("#edges = {}", edges.size());

    const unsigned int edges_size = edges.size();
    for (unsigned int i = 0; i < edges_size; i++) {
        double weight = -1;
        if (isCollapsable_cd1(edges[i][0], edges[i][1])) {
            weight = calEdgeLength(edges[i][0], edges[i][1]);
            if (isCollapsable_cd3(edges[i][0], edges[i][1], weight)) {
                ElementInQueue_ec ele(edges[i], weight);
                ec_queue.push(ele);
            }
        }
        if (isCollapsable_cd1(edges[i][1], edges[i][0])) {
            weight = weight == -1 ? calEdgeLength(edges[i][0], edges[i][1]) : weight;
            if (isCollapsable_cd3(edges[i][0], edges[i][1], weight)) {
                ElementInQueue_ec ele({{edges[i][1], edges[i][0]}}, weight);
                ec_queue.push(ele);
            }
        }
    }

    counter = 0;
    suc_counter = 0;
    breakdown_timing = {{0, 0, 0, 0, 0}};
}

void EdgeCollapser::collapse()
{
    tet_tss.assign(tets.size(), 0);
    int cnt = 0;
    logger().debug("edge queue size = {}", ec_queue.size());
    while (!ec_queue.empty()) {
        std::array<int, 2> v_ids = ec_queue.top().v_ids;
        double old_weight = ec_queue.top().weight;
        ec_queue.pop();

        if (!isEdgeValid(v_ids)) {
            continue;
        }

        // during operations, the length of edges in the queue may be changed
        // also, we need to eliminate the old edges, that is, the edges have an wrong/old weight in the queue
        double weight = calEdgeLength(v_ids[0], v_ids[1]);
        if (weight != old_weight || !isCollapsable_cd3(v_ids[0], v_ids[1], weight)) {
            continue;
        }

        while (!ec_queue.empty()) {
            std::array<int, 2> tmp_v_ids = ec_queue.top().v_ids;
            if (tmp_v_ids == v_ids)
                ec_queue.pop();
            else
                break;
        }

        igl_timer.start();
        int return_code = collapseAnEdge(v_ids[0], v_ids[1]);
        if (return_code == SUCCESS) {
            breakdown_timing[id_success] += igl_timer.getElapsedTime();
            suc_counter++;
            if (budget > 0) {
                budget--;
                if (budget == 0) return;
            }
        } else if (return_code == ENVELOP_SUC) {
            breakdown_timing[id_env_success] += igl_timer.getElapsedTime();
            suc_counter++;
            cnt++;
            if (budget > 0) {
                budget--;
                if (budget == 0) return;
            }
        } else {
            if (return_code == ENVELOP) {
                breakdown_timing[id_env_fail] += igl_timer.getElapsedTime();
            } else if (return_code == FLIP) {
                breakdown_timing[id_flip_fail] += igl_timer.getElapsedTime();
            } else {
                breakdown_timing[id_energy_fail] += igl_timer.getElapsedTime();
            }

            inf_es.push_back(v_ids);
            inf_e_tss.push_back(ts);
        }

        counter++;
    }
    logger().debug("executed: {} | success / fail: {} / {}", counter, suc_counter, inf_es.size());
    logger().debug("envelope accept = {}", envelop_accept_cnt);

    if (suc_counter == 0 || inf_es.size() == 0) {
        //// report timings
        // logger().debug("----");
        // for (int i = 0; i < breakdown_timing.size(); i++) {
        //     logger().debug("{}: {}s", breakdown_name[i], breakdown_timing[i]);
        // }
        // logger().debug("energy_time = {}", energy_time);

        return; // this is the way out of collapse
    }

    postProcess(); // recursive call. This function calles collapse() again.
}

void EdgeCollapser::postProcess()
{
    // logger().debug("postProcess!");
    counter = 0;
    suc_counter = 0;
    envelop_accept_cnt = 0;

    // you CANNOT sort it here!! every inf_es has a time stamp!!!
    //    std::sort(inf_es.begin(), inf_es.end());
    //    inf_es.erase(std::unique(inf_es.begin(), inf_es.end()), inf_es.end());

    igl_timer.start();
    std::vector<std::array<int, 2>> tmp_inf_es;
    const unsigned int inf_es_size = inf_es.size();
    tmp_inf_es.reserve(inf_es_size / 4.0 + 1);
    for (unsigned int i = 0; i < inf_es_size; i++) {
        if (!isEdgeValid(inf_es[i])) {
            continue;
        }
        double weight = calEdgeLength(inf_es[i][0], inf_es[i][1]);
        if (!isCollapsable_cd3(inf_es[i][0], inf_es[i][1], weight)) {
            continue;
        }

        bool is_recal = false;
        for (auto it = tet_vertices[inf_es[i][0]].conn_tets.begin();
             it != tet_vertices[inf_es[i][0]].conn_tets.end();
             it++) {
            if (tet_tss[*it] > inf_e_tss[i]) {
                is_recal = true;
                break;
            }
        }

        if (is_recal && isCollapsable_cd1(inf_es[i][0], inf_es[i][1])) {
            if (!isLocked_ui(inf_es[i])) {
                ElementInQueue_ec ele(inf_es[i], weight);
                ec_queue.push(ele);
            }
        } else
            tmp_inf_es.push_back(inf_es[i]);
    }

    std::sort(tmp_inf_es.begin(), tmp_inf_es.end());
    tmp_inf_es.erase(
        std::unique(tmp_inf_es.begin(), tmp_inf_es.end()),
        tmp_inf_es.end()); // it's better
    inf_es = tmp_inf_es;
    ts++;
    inf_e_tss = std::vector<int>(inf_es.size(), ts);

    breakdown_timing[id_postprocessing] += igl_timer.getElapsedTime();

    collapse();
}

int EdgeCollapser::collapseAnEdge(int v1_id, int v2_id)
{
    bool is_edge_too_short = false;
    bool is_edge_degenerate = false; // DZ: edge has length 0
    double length = (tet_vertices[v1_id].posf - tet_vertices[v2_id].posf).norm();
    if (length == 0) {
        is_edge_degenerate = true;
    }

    // check isolated
    if (tet_vertices[v1_id].is_on_surface && isIsolated(v1_id)) {
        tet_vertices[v1_id].is_on_surface = false;
        tet_vertices[v1_id].is_on_boundary = false;
        tet_vertices[v1_id].on_fixed_vertex = -1;
        tet_vertices[v1_id].on_face.clear();
        tet_vertices[v1_id].on_edge.clear();
    }
    if (!isBoundaryPoint(v1_id)) tet_vertices[v1_id].is_on_boundary = false;

    // check boundary
    if (tet_vertices[v1_id].is_on_boundary && !tet_vertices[v2_id].is_on_boundary)
        if (!is_edge_degenerate && isPointOutBoundaryEnvelop(tet_vertices[v2_id].posf)) {
            return ENVELOP;
        }

    // check envelop
    if (tet_vertices[v1_id].is_on_surface && !tet_vertices[v2_id].is_on_surface) {
        if (!is_edge_degenerate && isPointOutEnvelop(tet_vertices[v2_id].posf)) {
            return ENVELOP;
        }
    }

    // old_t_ids
    std::vector<int> old_t_ids;
    old_t_ids.reserve(tet_vertices[v1_id].conn_tets.size());
    for (auto it = tet_vertices[v1_id].conn_tets.begin(); it != tet_vertices[v1_id].conn_tets.end();
         it++)
        old_t_ids.push_back(*it);
    std::vector<bool> is_removed(old_t_ids.size(), false);

    // new_tets
    std::vector<std::array<int, 4>> new_tets;
    new_tets.reserve(old_t_ids.size());
    std::unordered_set<int> n12_v_ids;
    std::vector<int> n12_t_ids;
    for (int i = 0; i < old_t_ids.size(); i++) {
        auto it = std::find(tets[old_t_ids[i]].begin(), tets[old_t_ids[i]].end(), v2_id);
        if (it == tets[old_t_ids[i]].end()) {
            std::array<int, 4> t = tets[old_t_ids[i]];
            auto jt = std::find(t.begin(), t.end(), v1_id);
            *jt = v2_id;
            new_tets.push_back(t);
        } else {
            is_removed[i] = true;
            for (int j = 0; j < 4; j++)
                if (tets[old_t_ids[i]][j] != v1_id && tets[old_t_ids[i]][j] != v2_id)
                    n12_v_ids.insert(tets[old_t_ids[i]][j]);
            n12_t_ids.push_back(old_t_ids[i]);
        }
    }

    // check 2
    if (isFlip(new_tets)) {
        return FLIP;
    }
    std::vector<TetQuality> tet_qs;
    igl::Timer tmp_timer;
    tmp_timer.start();
    calTetQualities(new_tets, tet_qs);
    energy_time += tmp_timer.getElapsedTime();

    if (is_check_quality) {
        TetQuality old_tq, new_tq;
        getCheckQuality(old_t_ids, old_tq);
        getCheckQuality(tet_qs, new_tq);
        if (is_soft) old_tq.slim_energy = soft_energy;
        if (!tet_vertices[v1_id].is_rounded) // remove an unroundable vertex anyway
            new_tq.slim_energy = 0;
        if (!is_edge_degenerate && !new_tq.isBetterOrEqualThan(old_tq, state)) {
            return QUALITY;
        }
    }

    // check 2.5
    if (tet_vertices[v1_id].is_on_boundary) {
        Vector3r old_p = tet_vertices[v1_id].pos;
        Vector3d old_pf = tet_vertices[v1_id].posf;
        tet_vertices[v1_id].posf = tet_vertices[v2_id].posf;
        tet_vertices[v1_id].pos = tet_vertices[v2_id].pos;

        tet_vertices[v1_id].posf = old_pf;
        tet_vertices[v1_id].pos = old_p;
    }

    // check 3
    bool is_envelop_suc = false;
    if (state.eps != state.EPSILON_INFINITE && tet_vertices[v1_id].is_on_surface) {
        if (!is_edge_degenerate && !isCollapsable_epsilon(v1_id, v2_id)) {
            return ENVELOP;
        }
        is_envelop_suc = true;
        envelop_accept_cnt++;
        if (envelop_accept_cnt % 1000 == 0) logger().debug("1000 accepted!");
    }

    // real update
    if (tet_vertices[v1_id].is_on_boundary) tet_vertices[v2_id].is_on_boundary = true;

    std::vector<std::array<int, 2>> update_sf_t_ids(n12_t_ids.size(), std::array<int, 2>());
    if (tet_vertices[v1_id].is_on_surface || tet_vertices[v2_id].is_on_surface) {
        for (int i = 0; i < n12_t_ids.size(); i++) {
            for (int j = 0; j < 4; j++) {
                if (tets[n12_t_ids[i]][j] == v1_id || tets[n12_t_ids[i]][j] == v2_id) {
                    std::vector<int> ts;
                    getFaceConnTets(
                        tets[n12_t_ids[i]][(j + 1) % 4],
                        tets[n12_t_ids[i]][(j + 2) % 4],
                        tets[n12_t_ids[i]][(j + 3) % 4],
                        ts);

                    if (tets[n12_t_ids[i]][j] == v1_id)
                        update_sf_t_ids[i][1] = ts[0] != n12_t_ids[i] ? ts[0] : ts[1];
                    else
                        update_sf_t_ids[i][0] = ts[0] != n12_t_ids[i] ? ts[0] : ts[1];
                }
            }
        }
    }

    std::unordered_set<int> n1_v_ids;
    int cnt = 0;
    for (int i = 0; i < old_t_ids.size(); i++) {
        if (is_removed[i]) {
            t_is_removed[old_t_ids[i]] = true;
            for (int j = 0; j < 4; j++)
                if (tets[old_t_ids[i]][j] != v1_id && tets[old_t_ids[i]][j] != v2_id) {
                    tet_vertices[tets[old_t_ids[i]][j]].conn_tets.erase(std::find(
                        tet_vertices[tets[old_t_ids[i]][j]].conn_tets.begin(),
                        tet_vertices[tets[old_t_ids[i]][j]].conn_tets.end(),
                        old_t_ids[i]));
                }
            tet_vertices[v2_id].conn_tets.erase(std::find(
                tet_vertices[v2_id].conn_tets.begin(),
                tet_vertices[v2_id].conn_tets.end(),
                old_t_ids[i]));
        } else {
            tet_vertices[v2_id].conn_tets.insert(old_t_ids[i]);
            tet_qualities[old_t_ids[i]] = tet_qs[cnt];
            for (int j = 0; j < 4; j++) {
                if (tets[old_t_ids[i]][j] != v1_id)
                    n1_v_ids.insert(tets[old_t_ids[i]][j]); // n12_v_ids would still be inserted
            }
            tets[old_t_ids[i]] = new_tets[cnt];
            cnt++;
        }
    }

    if (tet_vertices[v1_id].is_on_surface || tet_vertices[v2_id].is_on_surface) {
        tet_vertices[v2_id].is_on_surface = true;

        bool is_check_isolated = false;
        for (int i = 0; i < n12_t_ids.size(); i++) {
            std::array<int, 2> is_sf_fs;
            std::vector<int> es;
            for (int j = 0; j < 4; j++) {
                if (tets[n12_t_ids[i]][j] != v1_id && tets[n12_t_ids[i]][j] != v2_id)
                    es.push_back(tets[n12_t_ids[i]][j]);
                else if (tets[n12_t_ids[i]][j] == v1_id)
                    is_sf_fs[0] = is_surface_fs[n12_t_ids[i]][j];
                else
                    is_sf_fs[1] = is_surface_fs[n12_t_ids[i]][j];
            }
            // be careful about the order!!

            if (is_sf_fs[0] == is_sf_fs[1] && is_sf_fs[0] == state.NOT_SURFACE) continue;
            if (is_sf_fs[0] == state.NOT_SURFACE) is_sf_fs[0] = 0;
            if (is_sf_fs[1] == state.NOT_SURFACE) is_sf_fs[1] = 0;

            int tmp0 = is_sf_fs[0];
            int tmp1 = is_sf_fs[1];
            is_sf_fs[0] += -tmp1;
            is_sf_fs[1] += -tmp0;

            for (int j = 0; j < 4; j++) {
                int v_id0 = tets[update_sf_t_ids[i][0]][j];
                if (v_id0 != v2_id && v_id0 != es[0] && v_id0 != es[1])
                    is_surface_fs[update_sf_t_ids[i][0]][j] = is_sf_fs[0];

                int v_id1 = tets[update_sf_t_ids[i][1]][j];
                if (v_id1 != v2_id && v_id1 != es[0] && v_id1 != es[1])
                    is_surface_fs[update_sf_t_ids[i][1]][j] = is_sf_fs[1];
            }
        }
    }

    v_is_removed[v1_id] = true;

    // update time stamps
    ts++;
    for (int i = 0; i < old_t_ids.size(); i++) {
        tet_tss[old_t_ids[i]] = ts;
    }

    // add new elements

    std::vector<int> n1_v_ids_vec, n12_v_ids_vec;
    n1_v_ids_vec.reserve(n1_v_ids.size());
    n12_v_ids_vec.reserve(n12_v_ids.size());
    for (auto it = n1_v_ids.begin(); it != n1_v_ids.end(); it++) n1_v_ids_vec.push_back(*it);
    for (auto it = n12_v_ids.begin(); it != n12_v_ids.end(); it++) n12_v_ids_vec.push_back(*it);
    std::sort(n1_v_ids_vec.begin(), n1_v_ids_vec.end());
    std::sort(n12_v_ids_vec.begin(), n12_v_ids_vec.end());
    n1_v_ids.clear();
    std::set_difference(
        n1_v_ids_vec.begin(),
        n1_v_ids_vec.end(),
        n12_v_ids_vec.begin(),
        n12_v_ids_vec.end(),
        std::inserter(n1_v_ids, n1_v_ids.end()));

    for (auto it = n1_v_ids.begin(); it != n1_v_ids.end(); it++) {
        double weight = -1;
        if (isCollapsable_cd1(v2_id, *it)) {
            weight = calEdgeLength(v2_id, *it);
            if (isCollapsable_cd3(v2_id, *it, weight)) {
                std::array<int, 2> e = {{v2_id, *it}};
                if (!isLocked_ui(e)) {
                    ElementInQueue_ec ele(e, weight);
                    ec_queue.push(ele);
                }
            }
        }
        if (isCollapsable_cd1(*it, v2_id)) {
            weight = weight == -1 ? calEdgeLength(*it, v2_id) : weight;
            if (isCollapsable_cd3(*it, v2_id, weight)) {
                std::array<int, 2> e = {{*it, v2_id}};
                if (!isLocked_ui(e)) {
                    ElementInQueue_ec ele(e, weight);
                    ec_queue.push(ele);
                }
            }
        }
    }

    if (is_envelop_suc) return ENVELOP_SUC;
    return SUCCESS;
}

/**
 * DZ: Check if v1 can be collapsed into v2 if v1 is on the bbox.
 */
bool EdgeCollapser::isCollapsable_cd1(int v1_id, int v2_id)
{
    // check the bbox tags //if the moved vertex is on the bbox
    bool is_movable = false;
    if (tet_vertices[v1_id].on_fixed_vertex < -1) return false;
    if (tet_vertices[v1_id].is_on_bbox && !tet_vertices[v2_id].is_on_bbox)
        return false;
    else if (tet_vertices[v1_id].is_on_bbox && tet_vertices[v2_id].is_on_bbox) {
        if (tet_vertices[v1_id].on_edge.size() == 0) { // inside the face
            is_movable = isHaveCommonEle(tet_vertices[v1_id].on_face, tet_vertices[v2_id].on_face);
        } else { // on the edge
            is_movable = isHaveCommonEle(tet_vertices[v1_id].on_edge, tet_vertices[v2_id].on_edge);
        }
        return is_movable;
    }

    return true;
}

/**
 * DZ: Check if edge is short enough to be collapsed.
 */
bool EdgeCollapser::isCollapsable_cd3(int v1_id, int v2_id, double weight)
{
    if (!is_limit_length) return true;

    double adaptive_scale =
        (tet_vertices[v1_id].adaptive_scale + tet_vertices[v2_id].adaptive_scale) / 2;
    if (weight < ideal_weight * adaptive_scale * adaptive_scale) return true;
    return false;
}

/**
 * DZ: Envelope check for edge (v1,v2)
 * - Find all surface triangles incident to v1 that are NOT incident to v2.
 * - Replace v1 with v2 and perform envelope check for these triangles
 */
bool EdgeCollapser::isCollapsable_epsilon(int v1_id, int v2_id)
{
    // get all surface triangles incident to v1
    std::vector<std::array<int, 3>> tri_ids;
    for (auto it = tet_vertices[v1_id].conn_tets.begin(); it != tet_vertices[v1_id].conn_tets.end();
         it++) {
        for (int j = 0; j < 4; j++) {
            if (tets[*it][j] != v1_id && is_surface_fs[*it][j] != state.NOT_SURFACE) {
                std::array<int, 3> tri = {
                    {tets[*it][(j + 1) % 4], tets[*it][(j + 2) % 4], tets[*it][(j + 3) % 4]}};
                std::sort(tri.begin(), tri.end());
                tri_ids.push_back(tri);
            }
        }
    }
    std::sort(tri_ids.begin(), tri_ids.end());
    tri_ids.erase(std::unique(tri_ids.begin(), tri_ids.end()), tri_ids.end());

    std::vector<std::array<Vector3d, 3>> tris;
    for (int i = 0; i < tri_ids.size(); i++) {
        // ignore triangles incident to v2
        if (std::find(tri_ids[i].begin(), tri_ids[i].end(), v2_id) != tri_ids[i].end()) continue;
        auto jt = std::find(tri_ids[i].begin(), tri_ids[i].end(), v1_id);
        *jt = v2_id; // replace v1 with v2
        std::array<Vector3d, 3> tri{
            tet_vertices[tri_ids[i][0]].posf,
            tet_vertices[tri_ids[i][1]].posf,
            tet_vertices[tri_ids[i][2]].posf};
        tris.push_back(tri);
    }

    /// note that tris.size() can be 0 when v1 is on the boundary of the surface!!!
    for (int i = 0; i < tris.size(); i++) {
        if (isFaceOutEnvelop(tris[i])) return false;
    }

    return true;
}

bool EdgeCollapser::isEdgeValid(const std::array<int, 2>& e)
{
    if (v_is_removed[e[0]] || v_is_removed[e[1]]) return false;
    return isHaveCommonEle(tet_vertices[e[0]].conn_tets, tet_vertices[e[1]].conn_tets);
}

} // namespace wmtk::components::tetwild::orig
