// This file is part of TetWild, a software for generating tetrahedral meshes.
//
// Copyright (C) 2018 Yixin Hu <yixin.hu@nyu.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//
// Created by Yixin Hu on 5/6/17.
//

#include "LocalOperations.h"
#include <wmtk/utils/AMIPS.h>
#include <numeric>
#include <wmtk/utils/Logger.hpp>
#include "Args.h"
#include "Common.h"

namespace wmtk::components::tetwild::orig {

void LocalOperations::outputInfo(int op_type, double time, bool is_log)
{
    logger().debug("outputing info");

    if (args.is_quiet) return;

    int cnt = 0;
    int r_cnt = 0;
    for (int i = 0; i < tet_vertices.size(); i++) {
        if (!v_is_removed[i]) {
            cnt++;
            if (tet_vertices[i].is_rounded) {
                r_cnt++;
            }
        }
    }


    logger().debug("# vertices = {}({}) {}(r)", cnt, tet_vertices.size(), r_cnt);

    cnt = 0;
    for (int i = 0; i < tets.size(); i++) {
        if (!t_is_removed[i]) cnt++;
    }
    logger().debug("# tets = {}({})", cnt, tets.size());
    logger().debug("# total operations = {}", counter);
    logger().debug("# accepted operations = {}", suc_counter);


    double max_slim_energy = 0, avg_slim_energy = 0;
    cnt = 0;

    for (int i = 0; i < tet_qualities.size(); i++) {
        if (t_is_removed[i]) continue;
        if (isTetLocked_ui(i)) continue;

        cnt++;
        if (tet_qualities[i].slim_energy > max_slim_energy)
            max_slim_energy = tet_qualities[i].slim_energy;
        avg_slim_energy += tet_qualities[i].slim_energy;
    }
}

/**
 * DZ: Return true if tet is inverted or degenerated.
 */
bool LocalOperations::isTetFlip(const std::array<int, 4>& t)
{
    bool is_rounded = true;
    for (int j = 0; j < 4; j++)
        if (!tet_vertices[t[j]].is_rounded) {
            is_rounded = false;
            break;
        }
    if (is_rounded) {
        igl::predicates::exactinit();
        auto res = igl::predicates::orient3d(
            tet_vertices[t[0]].posf,
            tet_vertices[t[1]].posf,
            tet_vertices[t[2]].posf,
            tet_vertices[t[3]].posf);
        int result;
        if (res == igl::predicates::Orientation::POSITIVE)
            result = 1;
        else if (res == igl::predicates::Orientation::NEGATIVE)
            result = -1;
        else
            result = 0;

        if (result < 0) // neg result == pos tet (tet origin from geogram delaunay)
            return false;
        return true;
    } else {
        Vector3r n = ((tet_vertices[t[1]].pos) - tet_vertices[t[0]].pos)
                         .cross((tet_vertices[t[2]].pos) - tet_vertices[t[0]].pos);
        Vector3r d = (tet_vertices[t[3]].pos) - tet_vertices[t[0]].pos;
        auto res = n.dot(d);
        if (res > 0) // predicates returns pos value: non-inverted
            return false;
        else
            return true;
    }
}

bool LocalOperations::isTetFlip(int t_id)
{
    return isTetFlip(tets[t_id]);
}

/**
 * DZ: Return true if any tet is inverted or degenerated.
 */
bool LocalOperations::isFlip(const std::vector<std::array<int, 4>>& new_tets)
{
    ////check orientation
    for (int i = 0; i < new_tets.size(); i++) {
        if (isTetFlip(new_tets[i])) return true;
    }

    return false;
}

void LocalOperations::getCheckQuality(const std::vector<TetQuality>& tet_qs, TetQuality& tq)
{
    double slim_sum = 0, slim_max = 0;
    for (int i = 0; i < tet_qs.size(); i++) {
        if (state.use_energy_max) {
            if (tet_qs[i].slim_energy > slim_max) slim_max = tet_qs[i].slim_energy;
        } else
            slim_sum += tet_qs[i].slim_energy * tet_qs[i].volume;
    }
    if (state.use_energy_max)
        tq.slim_energy = slim_max;
    else
        tq.slim_energy = slim_sum;
}

void LocalOperations::getCheckQuality(const std::vector<int>& t_ids, TetQuality& tq)
{
    double slim_sum = 0, slim_max = 0;
    for (int i = 0; i < t_ids.size(); i++) {
        if (state.use_energy_max) {
            if (tet_qualities[t_ids[i]].slim_energy > slim_max)
                slim_max = tet_qualities[t_ids[i]].slim_energy;
        } else
            slim_sum += tet_qualities[t_ids[i]].slim_energy * tet_qualities[t_ids[i]].volume;
    }
    if (state.use_energy_max)
        tq.slim_energy = slim_max;
    else
        tq.slim_energy = slim_sum;
}

void LocalOperations::getAvgMaxEnergy(double& avg_tq, double& max_tq)
{
    avg_tq = 0;
    max_tq = 0;
    int cnt = 0;
    for (unsigned int i = 0; i < tet_qualities.size(); i++) {
        if (t_is_removed[i]) continue;
        if (isTetLocked_ui(i)) continue;
        if (tet_qualities[i].slim_energy > max_tq) max_tq = tet_qualities[i].slim_energy;
        avg_tq += tet_qualities[i].slim_energy;
        cnt++;
    }
    avg_tq /= cnt;
    if (std::isinf(avg_tq)) avg_tq = state.MAX_ENERGY;
}

double LocalOperations::getMaxEnergy()
{
    double max_tq = 0;
    for (unsigned int i = 0; i < tet_qualities.size(); i++) {
        if (t_is_removed[i]) continue;
        if (isTetLocked_ui(i)) continue;
        if (tet_qualities[i].slim_energy > max_tq) max_tq = tet_qualities[i].slim_energy;
    }
    return max_tq;
}

double LocalOperations::getSecondMaxEnergy(double max_energy)
{
    double max_tq = 0;
    for (unsigned int i = 0; i < tet_qualities.size(); i++) {
        if (t_is_removed[i]) continue;
        if (tet_qualities[i].slim_energy == state.MAX_ENERGY) continue;
        if (isTetLocked_ui(i)) continue;
        if (tet_qualities[i].slim_energy > max_tq) max_tq = tet_qualities[i].slim_energy;
    }
    return max_tq;
}

double LocalOperations::getFilterEnergy(bool& is_clean_up)
{
    std::array<int, 11> buckets;
    for (int i = 0; i < 11; i++) buckets[i] = 0;
    for (unsigned int i = 0; i < tet_qualities.size(); i++) {
        if (t_is_removed[i]) continue;
        if (tet_qualities[i].slim_energy > args.filter_energy_thres - 1 + 1e10)
            buckets[10]++;
        else {
            for (int j = 0; j < 10; j++) {
                if (tet_qualities[i].slim_energy > args.filter_energy_thres - 1 + pow(10, j) &&
                    tet_qualities[i].slim_energy <= args.filter_energy_thres - 1 + pow(10, j + 1)) {
                    buckets[j]++;
                    break;
                }
            }
        }
    }

    std::array<int, 10> tmps1;
    std::array<int, 10> tmps2;
    for (int i = 0; i < 10; i++) {
        tmps1[i] = std::accumulate(buckets.begin(), buckets.begin() + i + 1, 0);
        tmps2[i] = std::accumulate(buckets.begin() + i + 1, buckets.end(), 0);
    }

    if (tmps1[0] >= tmps2[0]) {
        is_clean_up = (tmps2[5] > 0);
        return 8;
    }
    if (tmps1[8] <= tmps2[8]) return 1e11;

    for (int i = 0; i < 8; i++) {
        if (tmps1[i] < tmps2[i] && tmps1[i + 1] > tmps2[i + 1]) {
            return args.filter_energy_thres - 1 + 5 * pow(10, i + 1);
        }
    }

    return 8; // would never be execuate, it's fine
}

void LocalOperations::calTetQualities(
    const std::vector<std::array<int, 4>>& new_tets,
    std::vector<TetQuality>& tet_qs,
    bool all_measure)
{
    tet_qs.resize(new_tets.size());
    for (int i = 0; i < new_tets.size(); i++) {
        calTetQuality_AMIPS(new_tets[i], tet_qs[i]);
    }
}

double LocalOperations::calEdgeLength(const std::array<int, 2>& v_ids)
{
    return (tet_vertices[v_ids[0]].posf - tet_vertices[v_ids[1]].posf).norm();
}

double LocalOperations::calEdgeLength(int v1_id, int v2_id, bool is_over_refine)
{
    return (tet_vertices[v1_id].posf - tet_vertices[v2_id].posf).norm();
}

void LocalOperations::calTetQuality_AMIPS(const std::array<int, 4>& tet, TetQuality& t_quality)
{
    if (energy_type == state.ENERGY_AMIPS) {
        igl::predicates::exactinit();
        auto res = igl::predicates::orient3d(
            tet_vertices[tet[0]].posf,
            tet_vertices[tet[1]].posf,
            tet_vertices[tet[2]].posf,
            tet_vertices[tet[3]].posf);
        int result;
        if (res == igl::predicates::Orientation::POSITIVE)
            result = 1;
        else if (res == igl::predicates::Orientation::NEGATIVE)
            result = -1;
        else
            result = 0;

        // neg result == pos tet (tet origin from geogram delaunay)
        if (result >= 0) { // degenerate in floats
            t_quality.slim_energy = state.MAX_ENERGY;
        } else {
            std::array<double, 12> T;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 3; j++) {
                    T[i * 3 + j] = tet_vertices[tet[i]].posf[j];
                }
            }
            t_quality.slim_energy = wmtk::AMIPS_energy(T);
            if (std::isinf(t_quality.slim_energy) || std::isnan(t_quality.slim_energy))
                t_quality.slim_energy = state.MAX_ENERGY;
        }
    }
    if (std::isinf(t_quality.slim_energy) || std::isnan(t_quality.slim_energy) ||
        t_quality.slim_energy <= 0)
        t_quality.slim_energy = state.MAX_ENERGY;
}

bool LocalOperations::isEdgeOnSurface(int v1_id, int v2_id)
{
    if (!tet_vertices[v1_id].is_on_surface || !tet_vertices[v2_id].is_on_surface) return false;

    std::vector<int> t_ids;
    setIntersection(tet_vertices[v1_id].conn_tets, tet_vertices[v2_id].conn_tets, t_ids);
    assert(t_ids.size() != 0);
    return isEdgeOnSurface(v1_id, v2_id, t_ids);
}

bool LocalOperations::isEdgeOnBbox(int v1_id, int v2_id)
{
    if (!tet_vertices[v1_id].is_on_bbox || !tet_vertices[v2_id].is_on_bbox) return false;

    std::vector<int> t_ids;
    setIntersection(tet_vertices[v1_id].conn_tets, tet_vertices[v2_id].conn_tets, t_ids);
    return isEdgeOnBbox(v1_id, v2_id, t_ids);
}

bool LocalOperations::isEdgeOnSurface(int v1_id, int v2_id, const std::vector<int>& t_ids)
{
    for (int i = 0; i < t_ids.size(); i++) {
        for (int j = 0; j < 4; j++) {
            if (tets[t_ids[i]][j] != v1_id && tets[t_ids[i]][j] != v2_id) {
                if (is_surface_fs[t_ids[i]][j] != state.NOT_SURFACE) return true;
            }
        }
    }
    return false;
}

bool LocalOperations::isEdgeOnBbox(int v1_id, int v2_id, const std::vector<int>& t_ids)
{
    std::unordered_set<int> v_ids;
    for (int i = 0; i < t_ids.size(); i++) {
        for (int j = 0; j < 4; j++) {
            if (tets[t_ids[i]][j] != v1_id && tets[t_ids[i]][j] != v2_id) {
                v_ids.insert(tets[t_ids[i]][j]);
            }
        }
    }
    if (v_ids.size() != t_ids.size()) return true;
    return false;
}

/**
 * DZ: Check if edge (v1,v2) is on the surface boundary.
 *
 * An edge cannot be on the boundary if v1 or v2 is not marked as on boundary.
 * An edge is on the boundary if it has exactly one surface face incident.
 */
bool LocalOperations::isEdgeOnBoundary(int v1_id, int v2_id)
{
    if (state.is_mesh_closed) return false;

    if (!tet_vertices[v1_id].is_on_boundary || !tet_vertices[v2_id].is_on_boundary) return false;

    //    return true;

    int cnt = 0;
    for (int t_id : tet_vertices[v1_id].conn_tets) {
        std::array<int, 4> opp_js; // DZ: all vertices that are adjacent to v1 except for v2
        int ii = 0;
        for (int j = 0; j < 4; j++) {
            if (tets[t_id][j] == v1_id || tets[t_id][j] == v2_id) continue;
            opp_js[ii++] = j;
        }
        if (ii == 2) {
            // DZ: opp_js vertices form a tet together with v1,v2
            if (is_surface_fs[t_id][opp_js[0]] != state.NOT_SURFACE) cnt++;
            if (is_surface_fs[t_id][opp_js[1]] != state.NOT_SURFACE) cnt++;
            if (cnt > 2) return false;
        }
    }
    if (cnt == 2) // is boundary edge
        return true;

    return false;
}

bool LocalOperations::isFaceOutEnvelop(const std::array<Vector3d, 3>& tri)
{
    return geo_sf_tree.is_outside(tri);
}

bool LocalOperations::isPointOutEnvelop(const Vector3d& p)
{
    return geo_sf_tree.is_outside(p);
}

bool LocalOperations::isPointOutBoundaryEnvelop(const Vector3d& p)
{
    return geo_b_tree.is_outside(p);
}

bool LocalOperations::isTetOnSurface(int t_id)
{
    for (int i = 0; i < 4; i++) {
        if (is_surface_fs[t_id][i] != state.NOT_SURFACE) return false;
    }
    return true;
}

bool LocalOperations::isTetRounded(int t_id)
{
    for (int i = 0; i < 4; i++) {
        if (!tet_vertices[tets[t_id][i]].is_rounded) return false;
    }
    return true;
}

void LocalOperations::getFaceConnTets(int v1_id, int v2_id, int v3_id, std::vector<int>& t_ids)
{
    std::vector<int> v1, v2, v3, tmp;
    v1.reserve(tet_vertices[v1_id].conn_tets.size());
    for (auto it = tet_vertices[v1_id].conn_tets.begin(); it != tet_vertices[v1_id].conn_tets.end();
         it++)
        v1.push_back(*it);
    v2.reserve(tet_vertices[v2_id].conn_tets.size());
    for (auto it = tet_vertices[v2_id].conn_tets.begin(); it != tet_vertices[v2_id].conn_tets.end();
         it++)
        v2.push_back(*it);
    v3.reserve(tet_vertices[v3_id].conn_tets.size());
    for (auto it = tet_vertices[v3_id].conn_tets.begin(); it != tet_vertices[v3_id].conn_tets.end();
         it++)
        v3.push_back(*it);

    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    std::sort(v3.begin(), v3.end());

    std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(tmp));
    std::set_intersection(v3.begin(), v3.end(), tmp.begin(), tmp.end(), std::back_inserter(t_ids));
}

/**
 * DZ: True if a vertex is marked as on surface but has no surface face incident.
 */
bool LocalOperations::isIsolated(int v_id)
{
    for (auto it = tet_vertices[v_id].conn_tets.begin(); it != tet_vertices[v_id].conn_tets.end();
         it++) {
        for (int j = 0; j < 4; j++) {
            if (tets[*it][j] != v_id && is_surface_fs[*it][j] != state.NOT_SURFACE) return false;
        }
    }

    return true;
}

/**
 * DZ: Check if vertex has any boundary edge incident.
 */
bool LocalOperations::isBoundaryPoint(int v_id)
{
    if (state.is_mesh_closed) return false;
    // DZ: gather adjacent vertices that are marked as on boundary
    std::unordered_set<int> n_v_ids;
    for (int t_id : tet_vertices[v_id].conn_tets) {
        for (int j = 0; j < 4; j++)
            if (tets[t_id][j] != v_id && tet_vertices[tets[t_id][j]].is_on_boundary)
                n_v_ids.insert(tets[t_id][j]);
    }
    for (int n_v_id : n_v_ids) {
        if (isEdgeOnBoundary(n_v_id, v_id)) return true;
    }
    return false;
}

void LocalOperations::checkUnrounded()
{
    bool is_output = false;
    for (unsigned int i = 0; i < tet_vertices.size(); i++) {
        if (v_is_removed[i]) continue;
        if (!tet_vertices[i].is_rounded) {
            is_output = true;
            break;
        }
    }
    if (!is_output) return;

    std::ofstream of;
    of.open(state.working_dir + "unrounded_check.txt");
    int cnt_sf = 0;
    int cnt_b = 0;
    int cnt_all = 0;
    int cnt_sf1 = 0;
    std::vector<double> diss;
    for (unsigned int i = 0; i < tet_vertices.size(); i++) {
        if (v_is_removed[i]) continue;
        if (tet_vertices[i].is_rounded) continue;

        cnt_all++;

        if (tet_vertices[i].is_on_boundary) cnt_b++;
        if (tet_vertices[i].is_on_surface) {
            cnt_sf++;
            continue;
        }

        bool is_found = false;
        for (int t_id : tet_vertices[i].conn_tets) {
            for (int j = 0; j < 4; j++) {
                if (tets[t_id][j] == i) {
                    if (is_surface_fs[t_id][j] != state.NOT_SURFACE) {
                        cnt_sf1++;
                        is_found = true;
                    }
                    break;
                }
            }
            if (is_found) break;
        }
        if (is_found) continue;

        double dis = sqrt(geo_sf_tree.squared_distance(tet_vertices[i].posf));
        diss.push_back(dis);
    }

    of << "# all = " << cnt_all << std::endl;
    of << "# surface = " << cnt_sf << std::endl;
    of << "# boundary = " << cnt_b << std::endl;
    //    of<<"Is closed? "<<is_closed<<std::endl;
    of << "# connect to surface = " << cnt_sf1 << std::endl;
    of << "# non-surface = " << diss.size() << std::endl;
    for (double dis : diss) {
        of << dis << std::endl;
    }
}

bool LocalOperations::isLocked_ui(const std::array<int, 2>& e)
{
    return (tet_vertices[e[0]].is_locked || tet_vertices[e[1]].is_locked);
}

bool LocalOperations::isTetLocked_ui(int tid)
{
    //    return false;

    for (int j = 0; j < 4; j++)
        if (tet_vertices[tets[tid][j]].is_locked) return true;
    return false;
}

} // namespace wmtk::components::tetwild::orig
