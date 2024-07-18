// This file is part of fTetWild, a software for generating tetrahedral meshes.
//
// Copyright (C) 2019 Yixin Hu <yixin.hu@nyu.edu>
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "tetwild_simplification.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/base/resolve_path.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <polysolve/Utils.hpp>

#include <queue>
#include <unordered_set>

namespace wmtk::components {

using Vector3 = Eigen::Vector3d;
using Vector3i = Eigen::Vector3i;

static const double SCALAR_ZERO_2 = 1e-16;

class AABBWrapper
{
public:
    bool is_out(const std::array<Vector3, 3>& triangle) const { return false; }

    inline void project_to_sf(Vector3& p) const
    {
        // GEO::vec3 geo_p(p[0], p[1], p[2]);
        // GEO::vec3 nearest_p;
        // double sq_dist = std::numeric_limits<double>::max(); //??
        // sf_tree.nearest_facet(geo_p, nearest_p, sq_dist);
        // p[0] = nearest_p[0];
        // p[1] = nearest_p[1];
        // p[2] = nearest_p[2];
    }
};

class ElementInQueue
{
public:
    std::array<int, 2> v_ids;
    double weight;

    ElementInQueue() {}
    ElementInQueue(const std::array<int, 2>& ids, double w)
        : v_ids(ids)
        , weight(w)
    {}
};

struct cmp_s
{
    bool operator()(const ElementInQueue& e1, const ElementInQueue& e2)
    {
        if (e1.weight == e2.weight) return e1.v_ids < e2.v_ids;
        return e1.weight > e2.weight;
    }
};

struct cmp_l
{
    bool operator()(const ElementInQueue& e1, const ElementInQueue& e2)
    {
        if (e1.weight == e2.weight) return e1.v_ids > e2.v_ids;
        return e1.weight < e2.weight;
    }
};

inline int mod3(int j)
{
    return j % 3;
}

void set_intersection(
    const std::unordered_set<int>& s1,
    const std::unordered_set<int>& s2,
    std::vector<int>& v)
{
    if (s2.size() < s1.size()) {
        set_intersection(s2, s1, v);
        return;
    }
    v.clear();
    v.reserve(std::min(s1.size(), s2.size()));
    for (int x : s1) {
        if (s2.count(x)) {
            v.push_back(x);
        }
    }
    //    std::sort(v.begin(), v.end());
}

template <typename T>
void vector_unique(std::vector<T>& v)
{
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
}

bool is_out_envelope(const std::array<Vector3, 3>& vs, const AABBWrapper& tree)
{
    return tree.is_out(vs);
}

double get_angle_cos(const Vector3& p, const Vector3& p1, const Vector3& p2)
{
    Vector3 v1 = p1 - p;
    Vector3 v2 = p2 - p;
    double res = v1.dot(v2) / (v1.norm() * v2.norm());
    if (res > 1) return 1;
    if (res < -1) return -1;
    return res;
}


// void remove_duplicates(
//     std::vector<Vector3>& input_vertices,
//     std::vector<Vector3i>& input_faces,
//     std::vector<int>& input_tags,
//     const double duplicate_tol)
// {
//     //    std::vector<size_t> indices(input_vertices.size());
//     //    for(size_t i=0;i<input_vertices.size();i++)
//     //        indices[i] = i;
//     //
//     //    std::sort(indices.begin(), indices.end(), [&input_vertices](size_t i1, size_t i2) {
//     //        return std::make_tuple(input_vertices[i1][0], input_vertices[i1][1],
//     //        input_vertices[i1][2])
//     //               < std::make_tuple(input_vertices[i2][0], input_vertices[i2][1],
//     //               input_vertices[i2][2]);
//     //    });
//     //    indices.erase(std::unique(indices.begin(), indices.end(), [&input_vertices](size_t i1,
//     //    size_t i2) {
//     //        return std::make_tuple(input_vertices[i1][0], input_vertices[i1][1],
//     //        input_vertices[i1][2])
//     //               == std::make_tuple(input_vertices[i2][0], input_vertices[i2][1],
//     //               input_vertices[i2][2]);
//     //    }), indices.end());

//     MatrixXs V_tmp(input_vertices.size(), 3), V_in;
//     Eigen::MatrixXi F_tmp(input_faces.size(), 3), F_in;
//     for (int i = 0; i < input_vertices.size(); i++) V_tmp.row(i) = input_vertices[i];
//     for (int i = 0; i < input_faces.size(); i++) F_tmp.row(i) = input_faces[i];

//     //
//     Eigen::VectorXi IV, _;
//     igl::remove_duplicate_vertices(
//         V_tmp,
//         F_tmp,
//         duplicate_tol,
//         V_in,
//         IV,
//         _,
//         F_in);
//     //
//     for (int i = 0; i < F_in.rows(); i++) {
//         int j_min = 0;
//         for (int j = 1; j < 3; j++) {
//             if (F_in(i, j) < F_in(i, j_min)) j_min = j;
//         }
//         if (j_min == 0) continue;
//         int v0_id = F_in(i, j_min);
//         int v1_id = F_in(i, (j_min + 1) % 3);
//         int v2_id = F_in(i, (j_min + 2) % 3);
//         F_in.row(i) << v0_id, v1_id, v2_id;
//     }
//     F_tmp.resize(0, 0);
//     Eigen::VectorXi IF;
//     igl::unique_rows(F_in, F_tmp, IF, _);
//     F_in = F_tmp;
//     std::vector<int> old_input_tags = input_tags;
//     input_tags.resize(IF.rows());
//     for (int i = 0; i < IF.rows(); i++) {
//         input_tags[i] = old_input_tags[IF(i)];
//     }
//     //
//     if (V_in.rows() == 0 || F_in.rows() == 0) return false;

//     logger().info("remove duplicates: ");
//     logger().info("#v: {} -> {}", input_vertices.size(), V_in.rows());
//     logger().info("#f: {} -> {}", input_faces.size(), F_in.rows());

//     input_vertices.resize(V_in.rows());
//     input_faces.clear();
//     input_faces.reserve(F_in.rows());
//     old_input_tags = input_tags;
//     input_tags.clear();
//     for (int i = 0; i < V_in.rows(); i++) input_vertices[i] = V_in.row(i);
//     for (int i = 0; i < F_in.rows(); i++) {
//         if (F_in(i, 0) == F_in(i, 1) || F_in(i, 0) == F_in(i, 2) || F_in(i, 2) == F_in(i, 1))
//             continue;
//         if (i > 0 && (F_in(i, 0) == F_in(i - 1, 0) && F_in(i, 1) == F_in(i - 1, 2) &&
//                       F_in(i, 2) == F_in(i - 1, 1)))
//             continue;
//         // check area
//         Vector3 u = V_in.row(F_in(i, 1)) - V_in.row(F_in(i, 0));
//         Vector3 v = V_in.row(F_in(i, 2)) - V_in.row(F_in(i, 0));
//         Vector3 area = u.cross(v);
//         if (area.norm() / 2 <= duplicate_tol) continue;
//         input_faces.push_back(F_in.row(i));
//         input_tags.push_back(old_input_tags[i]);
//     }

//     return true;
// }

void collapsing(
    std::vector<Vector3>& input_vertices,
    std::vector<Vector3i>& input_faces,
    const AABBWrapper& tree,
    std::vector<bool>& v_is_removed,
    std::vector<bool>& f_is_removed,
    std::vector<std::unordered_set<int>>& conn_fs)
{
    std::vector<std::array<int, 2>> edges;
    edges.reserve(input_faces.size() * 3);
    for (size_t f_id = 0; f_id < input_faces.size(); ++f_id) {
        if (f_is_removed[f_id]) continue;

        const auto& f = input_faces[f_id];
        for (int j = 0; j < 3; j++) {
            std::array<int, 2> e = {{f[j], f[mod3(j + 1)]}};
            if (e[0] > e[1]) std::swap(e[0], e[1]);
            edges.push_back(e);
        }
    }
    vector_unique(edges);

    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_s> sm_queue;
    for (auto& e : edges) {
        double weight = (input_vertices[e[0]] - input_vertices[e[1]]).squaredNorm();
        sm_queue.push(ElementInQueue(e, weight));
        sm_queue.push(ElementInQueue(std::array<int, 2>({{e[1], e[0]}}), weight));
    }


    auto is_onering_clean = [&](int v_id) {
        std::vector<int> v_ids;
        v_ids.reserve(conn_fs[v_id].size() * 2);
        for (const auto& f_id : conn_fs[v_id]) {
            for (int j = 0; j < 3; j++) {
                if (input_faces[f_id][j] != v_id) v_ids.push_back(input_faces[f_id][j]);
            }
        }
        std::sort(v_ids.begin(), v_ids.end());

        if (v_ids.size() % 2 != 0) return false;
        for (int i = 0; i < v_ids.size(); i += 2) {
            if (v_ids[i] != v_ids[i + 1]) return false;
        }

        return true;
    };

    static const int SUC = 1;
    static const int FAIL_CLEAN = 0;
    static const int FAIL_FLIP = -1;
    static const int FAIL_ENV = -2;

    auto remove_an_edge = [&](int v1_id, int v2_id, const std::vector<int>& n12_f_ids) {
        if (!is_onering_clean(v1_id) || !is_onering_clean(v2_id)) return FAIL_CLEAN;

        //        std::unordered_set<int> new_f_ids;
        std::vector<int> new_f_ids;
        for (int f_id : conn_fs[v1_id]) {
            if (f_id != n12_f_ids[0] && f_id != n12_f_ids[1]) new_f_ids.push_back(f_id);
        }
        for (int f_id : conn_fs[v2_id]) {
            if (f_id != n12_f_ids[0] && f_id != n12_f_ids[1]) new_f_ids.push_back(f_id);
        }
        vector_unique(new_f_ids);

        // compute new point
        Vector3 p = (input_vertices[v1_id] + input_vertices[v2_id]) / 2;
        tree.project_to_sf(p);
        // GEO::vec3 geo_p(p[0], p[1], p[2]);
        // GEO::vec3 nearest_p;
        // double _;
        // tree.nearest_facet(geo_p, nearest_p, _);
        // p[0] = nearest_p[0];
        // p[1] = nearest_p[1];
        // p[2] = nearest_p[2];

        // computing normal for checking flipping
        for (int f_id : new_f_ids) {
            Vector3 old_nv =
                (input_vertices[input_faces[f_id][1]] - input_vertices[input_faces[f_id][2]])
                    .cross(
                        input_vertices[input_faces[f_id][0]] -
                        input_vertices[input_faces[f_id][2]]);

            for (int j = 0; j < 3; j++) {
                if (input_faces[f_id][j] == v1_id || input_faces[f_id][j] == v2_id) {
                    Vector3 new_nv = (input_vertices[input_faces[f_id][mod3(j + 1)]] -
                                      input_vertices[input_faces[f_id][mod3(j + 2)]])
                                         .cross(p - input_vertices[input_faces[f_id][mod3(j + 2)]]);
                    if (old_nv.dot(new_nv) <= 0) return FAIL_FLIP;
                    // check new tris' area
                    if (new_nv.norm() / 2 <= SCALAR_ZERO_2) return FAIL_FLIP;
                    break;
                }
            }
        }

        // check if go outside of envelop
        for (int f_id : new_f_ids) {
            for (int j = 0; j < 3; j++) {
                if (input_faces[f_id][j] == v1_id || input_faces[f_id][j] == v2_id) {
                    const std::array<Vector3, 3> tri = {
                        {p,
                         input_vertices[input_faces[f_id][mod3(j + 1)]],
                         input_vertices[input_faces[f_id][mod3(j + 2)]]}};
                    if (is_out_envelope(tri, tree)) return FAIL_ENV;
                    break;
                }
            }
        }

        // real update
        //        std::unordered_set<int> n_v_ids;//get this info before real update for later usage
        std::vector<int> n_v_ids; // get this info before real update for later usage
        for (int f_id : new_f_ids) {
            for (int j = 0; j < 3; j++) {
                if (input_faces[f_id][j] != v1_id && input_faces[f_id][j] != v2_id)
                    n_v_ids.push_back(input_faces[f_id][j]);
            }
        }
        vector_unique(n_v_ids);

        v_is_removed[v1_id] = true;
        input_vertices[v2_id] = p;
        for (int f_id : n12_f_ids) {
            f_is_removed[f_id] = true;
            for (int j = 0; j < 3; j++) { // rm conn_fs
                if (input_faces[f_id][j] != v1_id) {
                    conn_fs[input_faces[f_id][j]].erase(f_id);
                }
            }
        }
        for (int f_id : conn_fs[v1_id]) { // add conn_fs
            if (f_is_removed[f_id]) continue;
            conn_fs[v2_id].insert(f_id);
            for (int j = 0; j < 3; j++) {
                if (input_faces[f_id][j] == v1_id) input_faces[f_id][j] = v2_id;
            }
        }

        // push new edges into the queue
        for (int v_id : n_v_ids) {
            double weight = (input_vertices[v2_id] - input_vertices[v_id]).squaredNorm();
            sm_queue.push(ElementInQueue(std::array<int, 2>({{v2_id, v_id}}), weight));
            sm_queue.push(ElementInQueue(std::array<int, 2>({{v_id, v2_id}}), weight));
        }
        return SUC;
    };

    int cnt_suc = 0;
    int fail_clean = 0;
    int fail_flip = 0;
    int fail_env = 0;

    while (!sm_queue.empty()) {
        std::array<int, 2> v_ids = sm_queue.top().v_ids;
        double old_weight = sm_queue.top().weight;
        sm_queue.pop();

        if (v_is_removed[v_ids[0]] || v_is_removed[v_ids[1]]) continue;
        if (old_weight != (input_vertices[v_ids[0]] - input_vertices[v_ids[1]]).squaredNorm())
            continue;

        std::vector<int> n12_f_ids;
        set_intersection(conn_fs[v_ids[0]], conn_fs[v_ids[1]], n12_f_ids);
        if (n12_f_ids.size() != 2) continue;

        int res = remove_an_edge(v_ids[0], v_ids[1], n12_f_ids);
        if (res == SUC)
            cnt_suc++;
        else if (res == FAIL_CLEAN)
            fail_clean++;
        else if (res == FAIL_FLIP)
            fail_flip++;
        else if (res == FAIL_ENV)
            fail_env++;
    }

    logger().trace(
        "{} success, {} fail, {} flip, {} out of envelope",
        cnt_suc,
        fail_clean,
        fail_flip,
        fail_env);
}

void swapping(
    std::vector<Vector3>& input_vertices,
    std::vector<Vector3i>& input_faces,
    const AABBWrapper& tree,
    std::vector<bool>& v_is_removed,
    std::vector<bool>& f_is_removed,
    std::vector<std::unordered_set<int>>& conn_fs)
{
    std::vector<std::array<int, 2>> edges;
    edges.reserve(input_faces.size() * 6);
    for (int i = 0; i < input_faces.size(); i++) {
        if (f_is_removed[i]) continue;
        auto& f = input_faces[i];
        for (int j = 0; j < 3; j++) {
            std::array<int, 2> e = {{f[j], f[mod3(j + 1)]}};
            if (e[0] > e[1]) std::swap(e[0], e[1]);
            edges.push_back(e);
        }
    }
    vector_unique(edges);

    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_l> sm_queue;
    for (auto& e : edges) {
        double weight = (input_vertices[e[0]] - input_vertices[e[1]]).squaredNorm();
        sm_queue.push(ElementInQueue(e, weight));
        sm_queue.push(ElementInQueue(std::array<int, 2>({{e[1], e[0]}}), weight));
    }

    int cnt = 0;
    while (!sm_queue.empty()) {
        int v1_id = sm_queue.top().v_ids[0];
        int v2_id = sm_queue.top().v_ids[1];
        sm_queue.pop();

        std::vector<int> n12_f_ids;
        set_intersection(conn_fs[v1_id], conn_fs[v2_id], n12_f_ids);
        if (n12_f_ids.size() != 2) continue;

        std::array<int, 2> n_v_ids = {{-1, -1}};
        for (int j = 0; j < 3; j++) {
            if (n_v_ids[0] < 0 && input_faces[n12_f_ids[0]][j] != v1_id &&
                input_faces[n12_f_ids[0]][j] != v2_id)
                n_v_ids[0] = input_faces[n12_f_ids[0]][j];

            if (n_v_ids[1] < 0 && input_faces[n12_f_ids[1]][j] != v1_id &&
                input_faces[n12_f_ids[1]][j] != v2_id)
                n_v_ids[1] = input_faces[n12_f_ids[1]][j];
        }

        // check coplanar
        double cos_a0 =
            get_angle_cos(input_vertices[n_v_ids[0]], input_vertices[v1_id], input_vertices[v2_id]);
        double cos_a1 =
            get_angle_cos(input_vertices[n_v_ids[1]], input_vertices[v1_id], input_vertices[v2_id]);
        std::array<Vector3, 2> old_nvs;
        for (int f = 0; f < 2; f++) {
            auto& a = input_vertices[input_faces[n12_f_ids[f]][0]];
            auto& b = input_vertices[input_faces[n12_f_ids[f]][1]];
            auto& c = input_vertices[input_faces[n12_f_ids[f]][2]];
            old_nvs[f] = ((b - c).cross(a - c)).normalized();
        }
        if (cos_a0 > -0.999) { // maybe it's for avoiding numerical issue
            if (old_nvs[0].dot(old_nvs[1]) < 1 - 1e-6) // not coplanar
                continue;
        }

        // check inversion
        auto& old_nv = cos_a1 < cos_a0 ? old_nvs[0] : old_nvs[1];
        bool is_filp = false;
        for (int f_id : n12_f_ids) {
            auto& a = input_vertices[input_faces[f_id][0]];
            auto& b = input_vertices[input_faces[f_id][1]];
            auto& c = input_vertices[input_faces[f_id][2]];
            if (old_nv.dot(((b - c).cross(a - c)).normalized()) < 0) {
                is_filp = true;
                break;
            }
        }
        if (is_filp) continue;

        // check quality
        double cos_a0_new = get_angle_cos(
            input_vertices[v1_id],
            input_vertices[n_v_ids[0]],
            input_vertices[n_v_ids[1]]);
        double cos_a1_new = get_angle_cos(
            input_vertices[v2_id],
            input_vertices[n_v_ids[0]],
            input_vertices[n_v_ids[1]]);
        if (std::min(cos_a0_new, cos_a1_new) <= std::min(cos_a0, cos_a1)) continue;

        if (is_out_envelope(
                {{input_vertices[v1_id], input_vertices[n_v_ids[0]], input_vertices[n_v_ids[1]]}},
                tree) ||
            is_out_envelope(
                {{input_vertices[v2_id], input_vertices[n_v_ids[0]], input_vertices[n_v_ids[1]]}},
                tree)) {
            continue;
        }

        // real update
        for (int j = 0; j < 3; j++) {
            if (input_faces[n12_f_ids[0]][j] == v2_id) input_faces[n12_f_ids[0]][j] = n_v_ids[1];
            if (input_faces[n12_f_ids[1]][j] == v1_id) input_faces[n12_f_ids[1]][j] = n_v_ids[0];
        }
        conn_fs[v1_id].erase(n12_f_ids[1]);
        conn_fs[v2_id].erase(n12_f_ids[0]);
        conn_fs[n_v_ids[0]].insert(n12_f_ids[1]);
        conn_fs[n_v_ids[1]].insert(n12_f_ids[0]);
        cnt++;
    }

    logger().trace("{}  faces are swapped!!", cnt);
    return;
}


void simplify(
    std::vector<Vector3>& input_vertices,
    std::vector<Vector3i>& input_faces,
    const AABBWrapper& tree)
// , const double duplicate_tol)
{
    // remove_duplicates(input_vertices, input_faces, duplicate_tol);

    std::vector<bool> v_is_removed(input_vertices.size(), false);
    std::vector<bool> f_is_removed(input_faces.size(), false);

    std::vector<std::unordered_set<int>> conn_fs(input_vertices.size());

    for (int i = 0; i < input_faces.size(); i++) {
        for (int j = 0; j < 3; j++) conn_fs[input_faces[i][j]].insert(i);
    }

    double collapsing_time, swapping_time, cleanup_time;

    {
        POLYSOLVE_SCOPED_STOPWATCH("Collapsing", collapsing_time, logger());
        collapsing(input_vertices, input_faces, tree, v_is_removed, f_is_removed, conn_fs);
    }

    {
        POLYSOLVE_SCOPED_STOPWATCH("Swapping", swapping_time, logger());
        swapping(input_vertices, input_faces, tree, v_is_removed, f_is_removed, conn_fs);
    }

    // clean up vs, fs, v
    {
        POLYSOLVE_SCOPED_STOPWATCH("Cleanup", cleanup_time, logger());
        std::vector<int> map_v_ids(input_vertices.size(), -1);
        int cnt = 0;
        for (int i = 0; i < input_vertices.size(); i++) {
            if (v_is_removed[i] || conn_fs[i].empty()) continue;
            map_v_ids[i] = cnt;
            cnt++;
        }

        std::vector<Vector3> new_input_vertices(cnt);
        cnt = 0;
        for (int i = 0; i < input_vertices.size(); i++) {
            if (v_is_removed[i] || conn_fs[i].empty()) continue;
            new_input_vertices[cnt++] = input_vertices[i];
        }
        input_vertices = new_input_vertices;

        // f
        cnt = 0;
        for (int i = 0; i < input_faces.size(); i++) {
            if (f_is_removed[i]) continue;
            for (int j = 0; j < 3; j++) input_faces[i][j] = map_v_ids[input_faces[i][j]];
            cnt++;
        }

        std::vector<Vector3i> new_input_faces(cnt);
        cnt = 0;
        for (int i = 0; i < input_faces.size(); i++) {
            if (f_is_removed[i]) continue;
            new_input_faces[cnt] = input_faces[i];
            cnt++;
        }
        input_faces = new_input_faces;
    }

    // remove_duplicates(input_vertices, input_faces, input_tags, duplicate_tol);

    logger().debug("#v = {}", input_vertices.size());
    logger().debug("#f = {}", input_faces.size());
}

void tetwild_simplification(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    // duplicate_tol = SCALAR_ZERO * bbox_diag_length

    std::shared_ptr<Mesh> mesh = cache.read_mesh(j["input"]);
    // options.attributes.position
}


} // namespace wmtk::components