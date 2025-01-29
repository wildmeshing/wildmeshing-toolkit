// This file is part of fTetWild, a software for generating tetrahedral meshes.
//
// Copyright (C) 2019 Yixin Hu <yixin.hu@nyu.edu>
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "tetwild_simplification.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>


#include <igl/remove_duplicate_vertices.h>
#include <igl/unique_rows.h>

#include <fastenvelope/FastEnvelope.h>
#include <SimpleBVH/BVH.hpp>

#include <polysolve/Utils.hpp>

#include <queue>
#include <random>
#include <unordered_set>

#ifdef WMTK_WITH_TBB
#include <tbb/concurrent_unordered_set.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#endif

namespace wmtk::components {

using Vector3 = Eigen::Vector3d;
using Vector3i = Eigen::Vector3i;


class Random
{
public:
    static int next(int min, int max)
    {
        static std::mt19937 gen(42);
        int res = (gen() - std::mt19937::min()) /
                      double(std::mt19937::max() - std::mt19937::min()) * (max - min) +
                  min;

        return res;
    }

    template <typename T>
    static void shuffle(std::vector<T>& vec)
    {
        for (int i = vec.size() - 1; i > 0; --i) {
            using std::swap;
            const int index = next(0, i + 1);
            swap(vec[i], vec[index]);
        }
    }
};

inline int mod3(int j)
{
    return j % 3;
}

class AABBWrapper
{
public:
    AABBWrapper(
        const std::vector<Eigen::Vector3d>& vertices,
        const std::vector<Eigen::Vector3i>& faces,
        double envelope_size,
        double sampling_dist,
        bool use_sampling)
        : m_envelope_size2(envelope_size * envelope_size)
        , m_sampling_dist(sampling_dist)
        , m_use_sampling(use_sampling)
    {
        m_envelope = std::make_shared<fastEnvelope::FastEnvelope>(vertices, faces, envelope_size);
        m_bvh = std::make_shared<SimpleBVH::BVH>();

        Eigen::MatrixXd V(vertices.size(), 3);
        Eigen::MatrixXi F(faces.size(), 3);
        for (int i = 0; i < vertices.size(); i++) V.row(i) = vertices[i];
        for (int i = 0; i < faces.size(); i++) {
            F.row(i) = faces[i];
            m_facets.push_back({{V.row(F(i, 0)), V.row(F(i, 1)), V.row(F(i, 2))}});
        }

        m_bvh->init(V, F, SCALAR_ZERO);
    }

    bool is_out(const std::array<Vector3, 3>& triangle) const
    {
        if (m_use_sampling)
            return is_out_using_sampling(triangle);
        else
            return m_envelope->is_outside(triangle);
    }

    bool is_out_envelope(const SimpleBVH::VectorMax3d& p, int& prev_facet) const
    {
        SimpleBVH::VectorMax3d nearest_point;
        double sq_dist = std::numeric_limits<double>::max();

        if (prev_facet >= 0) {
            m_bvh->point_facet_distance(p, prev_facet, nearest_point, sq_dist);

            if (sq_dist < m_envelope_size2) return false;
        }


        // prev_facet = m_bvh->facet_in_envelope(p, m_envelope_size2, nearest_point, sq_dist);
        m_bvh->facet_in_envelope_with_hint(p, m_envelope_size2, prev_facet, nearest_point, sq_dist);

        return sq_dist > m_envelope_size2;
    }

    bool is_out_using_sampling(const std::array<Vector3, 3>& vs) const
    {
        static const double sqrt3_2 = std::sqrt(3) / 2;

        int prev_facet = -1;

        // first check 3 verts
        if (is_out_envelope(vs[0], prev_facet)) return true;
        if (is_out_envelope(vs[1], prev_facet)) return true;
        if (is_out_envelope(vs[2], prev_facet)) return true;
        ////////////////////

        std::array<double, 3> ls;
        for (int i = 0; i < 3; i++) {
            ls[i] = (vs[i] - vs[mod3(i + 1)]).squaredNorm();
        }

        auto min_max = std::minmax_element(ls.begin(), ls.end());
        const int min_i = min_max.first - ls.begin();
        const int max_i = min_max.second - ls.begin();

        double N = sqrt(ls[max_i]) / m_sampling_dist;

        // no sampling, we alrady checked the vertices
        if (N <= 1) {
            return false;
        }

        if (N == int(N)) N -= 1;

        const Eigen::Vector3d& v0 = vs[max_i];
        const Eigen::Vector3d& v1 = vs[mod3(max_i + 1)];
        const Eigen::Vector3d& v2 = vs[mod3(max_i + 2)];

        const Eigen::Vector3d v1v0 = v1 - v0;
        const Eigen::Vector3d v2v0 = v2 - v0;
        const Eigen::Vector3d v2v1 = v2 - v1;
        const Eigen::Vector3d n_v0v1 = v1v0.normalized();

        // start from 1, we already checked the vertices
        for (int n = 1; n < N; n++) {
            const Eigen::Vector3d p = v0 + n_v0v1 * m_sampling_dist * n;
            if (is_out_envelope(p, prev_facet)) return true;
        }


        // const double h = (v2v0.dot(v1v0) * v1v0 / ls[max_i] + v0 - v2).norm();
        const double h = (v2v0.dot(v1v0) * v1v0 / ls[max_i] - v2v0).norm();
        const int M = h / (sqrt3_2 * m_sampling_dist);
        // we already checked the vertices
        if (M < 1) {
            return false;
        }

        const Eigen::Vector3d n_v0v2 = v2v0.normalized();
        const Eigen::Vector3d n_v1v2 = v2v1.normalized();

        const double sin_v0 = (v2v0.cross(v1v0)).norm() / (v2v0.norm() * v1v0.norm());
        const double tan_v0 = (v2v0.cross(v1v0)).norm() / v2v0.dot(v1v0);
        // const double tan_v1 = ((v2 - v1).cross(v0 - v1)).norm() / (v2 - v1).dot(v0 - v1);
        const double tan_v1 = -(v2v1.cross(v1v0)).norm() / (v2v1).dot(v1v0);
        // const double sin_v1 = (v2v1.cross(v0 - v1)).norm() / ((v1 - v2).norm() * (v0 - v1).norm());
        const double sin_v1 = (v2v1.cross(v1v0)).norm() / (v2v1.norm() * v1v0.norm());

        for (int m = 1; m <= M; m++) {
            int n = sqrt3_2 / tan_v0 * m + 0.5;
            int n1 = sqrt3_2 / tan_v0 * m;
            if (m % 2 == 0 && n == n1) {
                n += 1;
            }
            const Eigen::Vector3d v0_m = v0 + m * sqrt3_2 * m_sampling_dist / sin_v0 * n_v0v2;
            const Eigen::Vector3d v1_m = v1 + m * sqrt3_2 * m_sampling_dist / sin_v1 * n_v1v2;
            if ((v0_m - v1_m).norm() <= m_sampling_dist) break;

            double delta_d = ((n + (m % 2) / 2.0) - m * sqrt3_2 / tan_v0) * m_sampling_dist;
            const Eigen::Vector3d v = v0_m + delta_d * n_v0v1;
            int N1 = (v - v1_m).norm() / m_sampling_dist;
            for (int i = 0; i <= N1; i++) {
                const Eigen::Vector3d p = v + i * n_v0v1 * m_sampling_dist;

                if (is_out_envelope(p, prev_facet)) return true;
            }
        }

        // sample edges
        N = sqrt(ls[mod3(max_i + 1)]) / m_sampling_dist;
        if (N > 1) {
            if (N == int(N)) N -= 1;

            for (int n = 1; n <= N; n++) {
                const Eigen::Vector3d p = v1 + n_v1v2 * m_sampling_dist * n;

                if (is_out_envelope(p, prev_facet)) return true;
            }
        }

        N = sqrt(ls[mod3(max_i + 2)]) / m_sampling_dist;
        if (N > 1) {
            if (N == int(N)) N -= 1;

            const Eigen::Vector3d n_v2v0 = -n_v0v2;
            for (int n = 1; n <= N; n++) {
                const Eigen::Vector3d p = v2 + n_v2v0 * m_sampling_dist * n;

                if (is_out_envelope(p, prev_facet)) return true;
            }
        }

        return false;
    }

    inline void project_to_sf(Vector3& p) const
    {
        SimpleBVH::VectorMax3d nearest_point;
        double sq_dist;

        m_bvh->nearest_facet(p, nearest_point, sq_dist);
        p[0] = nearest_point[0];
        p[1] = nearest_point[1];
        p[2] = nearest_point[2];
    }

private:
    std::shared_ptr<fastEnvelope::FastEnvelope> m_envelope = nullptr;
    std::shared_ptr<SimpleBVH::BVH> m_bvh = nullptr;
    const double m_envelope_size2;
    const double m_sampling_dist;
    const bool m_use_sampling;
    std::vector<std::array<SimpleBVH::VectorMax3d, 3>> m_facets;
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

#ifdef WMTK_WITH_TBB
void one_ring_edge_set(
    const std::vector<std::array<int, 2>>& edges,
    const std::vector<bool>& v_is_removed,
    const std::vector<bool>& f_is_removed,
    const std::vector<std::unordered_set<int>>& conn_fs,
    const std::vector<Vector3>& input_vertices,
    std::vector<int>& safe_set)
{
    std::vector<int> indices(edges.size());
    std::iota(std::begin(indices), std::end(indices), 0);
    Random::shuffle(indices);

    std::vector<bool> unsafe_face(f_is_removed.size(), false);
    safe_set.clear();
    for (const int e_id : indices) {
        const auto e = edges[e_id];
        if (v_is_removed[e[0]] || v_is_removed[e[1]]) continue;

        bool ok = true;


        for (const int f : conn_fs[e[0]]) {
            if (f_is_removed[f]) continue;

            if (unsafe_face[f]) {
                ok = false;
                break;
            }
        }
        if (!ok) continue;
        for (const int f : conn_fs[e[1]]) {
            if (f_is_removed[f]) continue;

            if (unsafe_face[f]) {
                ok = false;
                break;
            }
        }
        if (!ok) continue;

        safe_set.push_back(e_id);

        for (const int f : conn_fs[e[0]]) {
            if (f_is_removed[f]) continue;

            assert(!unsafe_face[f]);
            unsafe_face[f] = true;
        }
        for (const int f : conn_fs[e[1]]) {
            if (f_is_removed[f]) continue;

            // assert(!unsafe_face[f]);
            unsafe_face[f] = true;
        }
    }
}
#endif

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


void remove_duplicates(
    std::vector<Vector3>& input_vertices,
    std::vector<Vector3i>& input_faces,
    const double duplicate_tol)
{
    Eigen::MatrixXd V_tmp(input_vertices.size(), 3), V_in;
    Eigen::MatrixXi F_tmp(input_faces.size(), 3), F_in;
    for (int i = 0; i < input_vertices.size(); i++) V_tmp.row(i) = input_vertices[i];
    for (int i = 0; i < input_faces.size(); i++) F_tmp.row(i) = input_faces[i];

    Eigen::VectorXi IV, _;
    igl::remove_duplicate_vertices(V_tmp, F_tmp, duplicate_tol, V_in, IV, _, F_in);

    for (int i = 0; i < F_in.rows(); i++) {
        int j_min = 0;
        for (int j = 1; j < 3; j++) {
            if (F_in(i, j) < F_in(i, j_min)) j_min = j;
        }
        if (j_min == 0) continue;
        int v0_id = F_in(i, j_min);
        int v1_id = F_in(i, (j_min + 1) % 3);
        int v2_id = F_in(i, (j_min + 2) % 3);
        F_in.row(i) << v0_id, v1_id, v2_id;
    }
    F_tmp.resize(0, 0);
    Eigen::VectorXi IF;
    igl::unique_rows(F_in, F_tmp, IF, _);
    F_in = F_tmp;

    if (V_in.rows() == 0 || F_in.rows() == 0) return;

    logger().trace("remove duplicates: ");
    logger().trace("#v: {} -> {}", input_vertices.size(), V_in.rows());
    logger().trace("#f: {} -> {}", input_faces.size(), F_in.rows());

    input_vertices.resize(V_in.rows());
    input_faces.clear();
    input_faces.reserve(F_in.rows());
    for (int i = 0; i < V_in.rows(); i++) input_vertices[i] = V_in.row(i);
    for (int i = 0; i < F_in.rows(); i++) {
        if (F_in(i, 0) == F_in(i, 1) || F_in(i, 0) == F_in(i, 2) || F_in(i, 2) == F_in(i, 1))
            continue;
        if (i > 0 && (F_in(i, 0) == F_in(i - 1, 0) && F_in(i, 1) == F_in(i - 1, 2) &&
                      F_in(i, 2) == F_in(i - 1, 1)))
            continue;
        // check area
        Vector3 u = V_in.row(F_in(i, 1)) - V_in.row(F_in(i, 0));
        Vector3 v = V_in.row(F_in(i, 2)) - V_in.row(F_in(i, 0));
        Vector3 area = u.cross(v);
        if (area.norm() / 2 <= duplicate_tol) continue;
        input_faces.push_back(F_in.row(i));
    }
}

void collapsing(
    std::vector<Vector3>& input_vertices,
    std::vector<Vector3i>& input_faces,
    const AABBWrapper& tree,
    std::vector<bool>& v_is_removed,
    std::vector<bool>& f_is_removed,
    std::vector<std::unordered_set<int>>& conn_fs)
{
#ifdef WMTK_WITH_TBB
    std::vector<std::array<int, 2>> edges;
    tbb::concurrent_vector<std::array<int, 2>> edges_tbb;

    const auto build_edges = [&]() {
        edges.clear();
        edges.reserve(input_faces.size() * 3);

        edges_tbb.clear();
        edges_tbb.reserve(input_faces.size() * 3);

        tbb::parallel_for(size_t(0), input_faces.size(), [&](size_t f_id) {
            if (f_is_removed[f_id]) return;

            for (int j = 0; j < 3; j++) {
                std::array<int, 2> e = {{input_faces[f_id][j], input_faces[f_id][mod3(j + 1)]}};
                if (e[0] > e[1]) std::swap(e[0], e[1]);
                edges_tbb.push_back(e);
            }
        });

        edges.reserve(edges_tbb.size());
        edges.insert(edges.end(), edges_tbb.begin(), edges_tbb.end());
        assert(edges_tbb.size() == edges.size());
        tbb::parallel_sort(edges.begin(), edges.end());

        edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
    };
#else
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
#endif

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
#ifndef WMTK_WITH_TBB
            for (int j = 0; j < 3; j++) { // rm conn_fs
                if (input_faces[f_id][j] != v1_id) {
                    conn_fs[input_faces[f_id][j]].erase(f_id);
                }
            }
#endif
        }
        for (int f_id : conn_fs[v1_id]) { // add conn_fs
            if (f_is_removed[f_id]) continue;
            conn_fs[v2_id].insert(f_id);
            for (int j = 0; j < 3; j++) {
                if (input_faces[f_id][j] == v1_id) input_faces[f_id][j] = v2_id;
            }
        }

#ifndef WMTK_WITH_TBB
        // push new edges into the queue
        for (int v_id : n_v_ids) {
            double weight = (input_vertices[v2_id] - input_vertices[v_id]).squaredNorm();
            sm_queue.push(ElementInQueue(std::array<int, 2>({{v2_id, v_id}}), weight));
            sm_queue.push(ElementInQueue(std::array<int, 2>({{v_id, v2_id}}), weight));
        }
#endif
        return SUC;
    };


#ifdef WMTK_WITH_TBB
    std::atomic<int> cnt(0);
    int cnt_suc = 0;
    int fail_clean = -1;
    int fail_flip = -1;
    int fail_env = -1;

    const int stopping = input_vertices.size() / 10000.;

    std::vector<int> safe_set;
    do {
        build_edges();
        one_ring_edge_set(edges, v_is_removed, f_is_removed, conn_fs, input_vertices, safe_set);
        cnt = 0;

        tbb::parallel_for(size_t(0), safe_set.size(), [&](size_t i) {
            //        for (int i = 0; i < safe_set.size(); i++) {
            std::array<int, 2>& v_ids = edges[safe_set[i]];

            //            if (v_is_removed[v_ids[0]] || v_is_removed[v_ids[1]])
            //                return;

            std::vector<int> n12_f_ids;
            set_intersection(conn_fs[v_ids[0]], conn_fs[v_ids[1]], n12_f_ids);

            if (n12_f_ids.size() != 2) return;
            //                continue;

            int res = remove_an_edge(v_ids[0], v_ids[1], n12_f_ids);
            if (res == SUC) cnt++;
        });
        //        }

        // cleanup conn_fs
        tbb::parallel_for(size_t(0), conn_fs.size(), [&](size_t i) {
            //        for (int i = 0; i < conn_fs.size(); i++) {
            if (v_is_removed[i])
                //                continue;
                return;
            std::vector<int> r_f_ids;
            for (int f_id : conn_fs[i]) {
                if (f_is_removed[f_id]) r_f_ids.push_back(f_id);
            }
            for (int f_id : r_f_ids) conn_fs[i].erase(f_id);
            //        }
        });

        cnt_suc += cnt;
    } while (cnt > stopping);

#else
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
#endif

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


nlohmann::json simplify(
    std::vector<Vector3>& input_vertices,
    std::vector<Vector3i>& input_faces,
    const AABBWrapper& tree,
    const double duplicate_tol)
{
    remove_duplicates(input_vertices, input_faces, duplicate_tol);

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

    remove_duplicates(input_vertices, input_faces, duplicate_tol);

    logger().debug("#v = {}", input_vertices.size());
    logger().debug("#f = {}", input_faces.size());

    nlohmann::json out;
    out["collapsing_time"] = collapsing_time;
    out["swapping_time"] = swapping_time;
    out["cleanup_time"] = cleanup_time;

    return out;
}

std::tuple<std::shared_ptr<TriMesh>, nlohmann::json> tetwild_simplification(
    const TriMesh& mesh,
    const std::string& postion_attr_name,
    double main_eps,
    bool relative,
    double duplicate_tol,
    bool use_sampling)
{
    wmtk::utils::EigenMatrixWriter writer;
    mesh.serialize(writer);

    Eigen::MatrixXd V;
    Eigen::MatrixX<int64_t> F;

    writer.get_double_matrix(postion_attr_name, PrimitiveType::Vertex, V);
    writer.get_FV_matrix(F);

    Eigen::VectorXd bmin(V.cols());
    bmin.setConstant(std::numeric_limits<double>::max());
    Eigen::VectorXd bmax(V.cols());
    bmax.setConstant(std::numeric_limits<double>::lowest());

    std::vector<Eigen::Vector3d> vertices(V.rows());
    std::vector<Eigen::Vector3i> faces(F.rows());
    for (int64_t i = 0; i < V.rows(); i++) {
        vertices[i] = V.row(i);
        for (int64_t d = 0; d < bmax.size(); ++d) {
            bmin[d] = std::min(bmin[d], vertices[i][d]);
            bmax[d] = std::max(bmax[d], vertices[i][d]);
        }
    }
    for (int64_t i = 0; i < F.rows(); i++) faces[i] = F.row(i).cast<int>();

    const double bbdiag = (bmax - bmin).norm();

    if (relative) main_eps *= bbdiag;

    if (duplicate_tol < 0) duplicate_tol = SCALAR_ZERO;
    if (relative) duplicate_tol *= bbdiag;

    const double envelope_size = 2 * main_eps * (9 - 2 * sqrt(3)) / 25.0;
    const double sampling_dist = (8.0 / 15.0) * main_eps;

    AABBWrapper tree(vertices, faces, envelope_size, sampling_dist, use_sampling);

    auto out = simplify(vertices, faces, tree, duplicate_tol);

    V.resize(vertices.size(), 3);
    F.resize(faces.size(), 3);
    for (int64_t i = 0; i < V.rows(); i++) V.row(i) = vertices[i];
    for (int64_t i = 0; i < F.rows(); i++) F.row(i) = faces[i].cast<int64_t>();

    auto res = std::make_shared<TriMesh>();
    res->initialize(F);
    mesh_utils::set_matrix_attribute(V, postion_attr_name, PrimitiveType::Vertex, *res);

    return {res, out};
}


} // namespace wmtk::components