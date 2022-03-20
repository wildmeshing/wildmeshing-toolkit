#include "TetWild.h"

#include <cstddef>
#include <wmtk/utils/Delaunay.hpp>
#include "wmtk/TetMesh.h"
#include "wmtk/auto_table.hpp"
#include "wmtk/utils/GeoUtils.h"
#include "wmtk/utils/Logger.hpp"

#include <igl/remove_duplicate_vertices.h>
#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>

#include <atomic>
#include <fstream>
#include <random>
#include <stdexcept>
#include <unordered_set>

bool tetwild::TetWild::InputSurface::remove_duplicates(double diag_l)
{
    Eigen::MatrixXd V_tmp(vertices.size(), 3), V_in;
    Eigen::MatrixXi F_tmp(faces.size(), 3), F_in;
    for (int i = 0; i < vertices.size(); i++) V_tmp.row(i) = vertices[i];
    for (int i = 0; i < faces.size(); i++)
        F_tmp.row(i) << faces[i][0], faces[i][1], faces[i][2]; // note: using int here

    //
    Eigen::VectorXi IV, _;
    igl::remove_duplicate_vertices(V_tmp, F_tmp, SCALAR_ZERO * diag_l, V_in, IV, _, F_in);
    //
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
    //    std::vector<int> old_input_tags = input_tags;
    //    input_tags.resize(IF.rows());
    //    for (int i = 0; i < IF.rows(); i++) {
    //        input_tags[i] = old_input_tags[IF(i)];
    //    }
    //
    if (V_in.rows() == 0 || F_in.rows() == 0) return false;

    wmtk::logger().info("remove duplicates: ");
    wmtk::logger().info("#v: {} -> {}", vertices.size(), V_in.rows());
    wmtk::logger().info("#f: {} -> {}", faces.size(), F_in.rows());

    std::vector<Vector3d> out_vertices;
    std::vector<std::array<size_t, 3>> out_faces;
    out_vertices.resize(V_in.rows());
    out_faces.reserve(F_in.rows());
    //    old_input_tags = input_tags;
    //    input_tags.clear();
    for (int i = 0; i < V_in.rows(); i++) out_vertices[i] = V_in.row(i);

    for (int i = 0; i < F_in.rows(); i++) {
        if (F_in(i, 0) == F_in(i, 1) || F_in(i, 0) == F_in(i, 2) || F_in(i, 2) == F_in(i, 1))
            continue;
        if (i > 0 && (F_in(i, 0) == F_in(i - 1, 0) && F_in(i, 1) == F_in(i - 1, 2) &&
                      F_in(i, 2) == F_in(i - 1, 1)))
            continue;
        // check area
        Vector3d u = V_in.row(F_in(i, 1)) - V_in.row(F_in(i, 0));
        Vector3d v = V_in.row(F_in(i, 2)) - V_in.row(F_in(i, 0));
        Vector3d area = u.cross(v);
        if (area.norm() / 2 <= SCALAR_ZERO * diag_l) continue;
        out_faces.push_back({{(size_t)F_in(i, 0), (size_t)F_in(i, 1), (size_t)F_in(i, 2)}});
        //        input_tags.push_back(old_input_tags[i]);
    }

    vertices = out_vertices;
    faces = out_faces;

    return true;
}

void tetwild::TetWild::init_from_delaunay_box_mesh(const std::vector<Eigen::Vector3d>& vertices)
{
    ///points for delaunay
    std::vector<wmtk::Point3D> points(vertices.size());
    for (int i = 0; i < vertices.size(); i++) {
        for (int j = 0; j < 3; j++) points[i][j] = vertices[i][j];
    }
    ///box
    double delta = m_params.diag_l / 15.0;
    Vector3d box_min(m_params.min[0] - delta, m_params.min[1] - delta, m_params.min[2] - delta);
    Vector3d box_max(m_params.max[0] + delta, m_params.max[1] + delta, m_params.max[2] + delta);
    int Nx = std::max(2, int((box_max[0] - box_min[0]) / delta));
    int Ny = std::max(2, int((box_max[1] - box_min[1]) / delta));
    int Nz = std::max(2, int((box_max[2] - box_min[2]) / delta));
    for (double i = 0; i <= Nx; i++) {
        for (double j = 0; j <= Ny; j++) {
            for (double k = 0; k <= Nz; k++) {
                Vector3d p(
                    box_min[0] * (1 - i / Nx) + box_max[0] * i / Nx,
                    box_min[1] * (1 - j / Ny) + box_max[1] * j / Ny,
                    box_min[2] * (1 - k / Nz) + box_max[2] * k / Nz);

                if (i == 0) p[0] = box_min[0];
                if (i == Nx) p[0] = box_max[0];
                if (j == 0) p[1] = box_min[1];
                if (j == Ny) p[1] = box_max[1];
                if (k == 0) p[2] = box_min[2];
                if (k == Nz) // note: have to do, otherwise the value would be slightly different
                    p[2] = box_max[2];

                if (!m_envelope.is_outside(p)) continue;
                points.push_back({{p[0], p[1], p[2]}});
            }
        }
    }
    m_params.box_min = box_min;
    m_params.box_max = box_max;

    ///delaunay
    auto tets = wmtk::delaunay3D_conn(points);
    wmtk::logger().info("tets.size() {}", tets.size());

    // conn
    init(points.size(), tets);
    // attr
    m_vertex_attribute.m_attributes.resize(points.size());
    m_tet_attribute.m_attributes.resize(tets.size());
    m_face_attribute.m_attributes.resize(tets.size() * 4);
    m_edge_attribute.m_attributes.resize(tets.size() * 6);
    for (int i = 0; i < m_vertex_attribute.m_attributes.size(); i++) {
        m_vertex_attribute[i].m_pos = Vector3r(points[i][0], points[i][1], points[i][2]);
        m_vertex_attribute[i].m_posf = Vector3d(points[i][0], points[i][1], points[i][2]);
    }
}

void tetwild::TetWild::match_insertion_faces(
    const std::vector<Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces,
    tbb::concurrent_vector<bool>& is_matched)
{
    is_matched.resize(faces.size(), false);

    std::map<std::array<size_t, 3>, size_t> map_surface;
    for (size_t i = 0; i < faces.size(); i++) {
        auto f = faces[i];
        std::sort(f.begin(), f.end());
        map_surface[f] = i;
    }
    for (size_t i = 0; i < m_tet_attribute.m_attributes.size(); i++) {
        Tuple t = tuple_from_tet(i);
        auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            std::array<size_t, 3> f = {
                {vs[(j + 1) % 4].vid(*this),
                 vs[(j + 2) % 4].vid(*this),
                 vs[(j + 3) % 4].vid(*this)}};
            std::sort(f.begin(), f.end());
            auto it = map_surface.find(f);
            if (it != map_surface.end()) {
                auto fid = it->second;
                triangle_insertion_helper.tet_face_tags[f].push_back(fid);
                is_matched[fid] = true;
            }
        }
    }
}

bool tetwild::TetWild::triangle_insertion_before(const std::vector<Tuple>& faces)
{
    triangle_insertion_local_cache.local().old_face_vids.clear(); // note: reset local vars

    for (auto& loc : faces) {
        auto vs = get_face_vertices(loc);
        std::array<size_t, 3> f = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
        std::sort(f.begin(), f.end());

        triangle_insertion_local_cache.local().old_face_vids.push_back(f);
    }

    return true;
}

bool tetwild::TetWild::triangle_insertion_after(
    const std::vector<std::vector<Tuple>>& new_faces)
{
    auto& tet_face_tags = triangle_insertion_helper.tet_face_tags;

    /// remove old_face_vids from tet_face_tags, and map tags to new faces
    // assert(new_faces.size() == triangle_insertion_local_cache.local().old_face_vids.size() + 1);

    for (int i = 0; i < new_faces.size(); i++) {
        if (new_faces[i].empty()) continue;

        // note: erase old tag and then add new -- old and new can be the same face
        std::vector<int> tags;
        if (i < triangle_insertion_local_cache.local().old_face_vids.size()) {
            auto& old_f = triangle_insertion_local_cache.local().old_face_vids[i];
            if (tet_face_tags.find(old_f) != tet_face_tags.end() && !tet_face_tags[old_f].empty()) {
                tags = tet_face_tags[old_f];
                tet_face_tags[old_f] = {};
            }
        } else
            tags = {triangle_insertion_local_cache.local().face_id};

        if (tags.empty()) continue;

        for (auto& loc : new_faces[i]) {
            auto vs = get_face_vertices(loc);
            std::array<size_t, 3> f = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
            std::sort(f.begin(), f.end());
            tet_face_tags[f] = tags;
        }
    }
    return true;
}

auto prepare_intersect_info = [](wmtk::TetMesh& m,
                                 const std::vector<Eigen::Vector3d>& vertices,
                                 const std::array<size_t, 3>& face_v,
                                 size_t face_id,
                                 auto& tet_face_tags,
                                 const auto& check_triangle_acquire,
                                 const auto& check_tet_acquire,
                                 const auto& vertex_pos_r) {
    using namespace tetwild;
    using Tuple = wmtk::TetMesh::Tuple;
    constexpr int EMPTY_INTERSECTION = 0;
    constexpr int TRI_INTERSECTION = 1;
    constexpr int PLN_INTERSECTION = 2;

    static constexpr std::array<std::array<int, 2>, 6> map_leid2lfids = {
        {{{0, 2}}, {{0, 3}}, {{0, 1}}, {{1, 2}}, {{2, 3}}, {{1, 3}}}};
    static constexpr std::array<std::array<int, 3>, 4> map_lvid2lfids = {
        {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 1, 3}}, {{1, 2, 3}}}};
    static constexpr std::array<std::array<int, 3>, 4> local_faces = {
        {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 1, 3}}, {{1, 2, 3}}}};
    static constexpr std::array<std::array<int, 2>, 6> local_edges = {
        {{{0, 1}}, {{1, 2}}, {{0, 2}}, {{0, 3}}, {{1, 3}}, {{2, 3}}}};

    bool success_flag = false;
    std::set<size_t> visited;

    std::array<Vector3r, 3> tri = {
        {to_rational(vertices[face_v[0]]),
         to_rational(vertices[face_v[1]]),
         to_rational(vertices[face_v[2]])}};
    std::array<Vector3d, 3> tri_d = {
        {vertices[face_v[0]], vertices[face_v[1]], vertices[face_v[2]]}};
    //
    Vector3r tri_normal = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
    //
    Vector3d tri_normal_d = (tri_d[1] - tri_d[0]).cross(tri_d[2] - tri_d[0]);
    std::array<Vector2r, 3> tri2;
    int squeeze_to_2d_dir = wmtk::project_triangle_to_2d(tri, tri2);

    std::vector<Tuple> intersected_tets;
    std::map<std::array<size_t, 2>, std::tuple<int, Vector3r, size_t, int>> map_edge2point;
    std::map<std::array<size_t, 3>, bool> map_face2intersected;
    // e = (e0, e1) ==> (intersection_status, intersection_point, edge's_tid,
    // local_eid_in_tet)
    std::set<std::array<size_t, 2>> intersected_tet_edges;
    //
    std::queue<Tuple> tet_queue;
    //


    if (!check_triangle_acquire(face_v)) {
        return std::tuple(success_flag, intersected_tets, map_edge2point);
    }

    for (int j = 0; j < 3; j++) {
        auto loc = m.tuple_from_vertex(face_v[j]);
        auto conn_tets = m.get_one_ring_tets_for_vertex(loc);
        for (const auto& t : conn_tets) {
            if (visited.find(t.tid(m)) != visited.end()) continue;
            tet_queue.push(t);
            visited.insert(t.tid(m));
        }
    }
    //
    auto is_seg_cut_tri_2 = [](const std::array<Vector2r, 2>& seg2,
                               const std::array<Vector2r, 3>& tri2) {
        // overlap == seg has one endpoint inside tri OR seg intersect with tri edges
        bool is_inside = wmtk::is_point_inside_triangle(seg2[0], tri2) ||
                         wmtk::is_point_inside_triangle(seg2[1], tri2);
        if (is_inside) {
            return true;
        } else {
            for (int j = 0; j < 3; j++) {
                apps::Rational _;
                std::array<Vector2r, 2> tri_seg2 = {{tri2[j], tri2[(j + 1) % 3]}};
                bool is_intersected =
                    wmtk::open_segment_open_segment_intersection_2d(seg2, tri_seg2, _);
                if (is_intersected) {
                    return true;
                }
            }
        }
        return false;
    };
    //

    // BFS
    while (!tet_queue.empty()) {
        auto tet = tet_queue.front();
        tet_queue.pop();

        std::array<bool, 4> is_tet_face_intersected = {{false, false, false, false}};

        /// check vertices position
        int cnt_pos = 0;
        int cnt_neg = 0;
        std::map<size_t, int> vertex_sides;
        std::vector<int> coplanar_f_lvids;
        //
        auto vs = m.oriented_tet_vertices(tet);

        auto retry_flag = !check_tet_acquire(std::vector<Tuple>{{tet}});
        if (retry_flag) {
            return std::tuple(success_flag, intersected_tets, map_edge2point);
        }

        std::array<size_t, 4> vertex_vids;
        for (int j = 0; j < 4; j++) {
            vertex_vids[j] = vs[j].vid(m);
            Vector3r dir = vertex_pos_r(vertex_vids[j]) - tri[0];
            auto side = dir.dot(tri_normal);
            if (side > 0) {
                cnt_pos++;
                vertex_sides[vertex_vids[j]] = 1;
            } else if (side < 0) {
                cnt_neg++;
                vertex_sides[vertex_vids[j]] = -1;
            } else {
                coplanar_f_lvids.push_back(j);
                vertex_sides[vertex_vids[j]] = 0;
            }
        }
        //
        if (coplanar_f_lvids.size() == 1) {
            int lvid = coplanar_f_lvids[0];
            int vid = vertex_vids[lvid];
            auto p = wmtk::project_point_to_2d(vertex_pos_r(vid), squeeze_to_2d_dir);
            bool is_inside = wmtk::is_point_inside_triangle(p, tri2);
            //
            if (is_inside) {
                auto conn_tets = m.get_one_ring_tets_for_vertex(vs[lvid]);
                for (auto& t : conn_tets) {
                    if (visited.count(t.tid(m))) continue;
                    visited.insert(t.tid(m));
                    tet_queue.push(t);
                }
            }
        } else if (coplanar_f_lvids.size() == 2) {
            std::array<Vector2r, 2> seg2;
            seg2[0] = wmtk::project_point_to_2d(
                vertex_pos_r(vertex_vids[coplanar_f_lvids[0]]),
                squeeze_to_2d_dir);
            seg2[1] = wmtk::project_point_to_2d(
                vertex_pos_r(vertex_vids[coplanar_f_lvids[1]]),
                squeeze_to_2d_dir);
            if (is_seg_cut_tri_2(seg2, tri2)) {
                std::array<int, 2> le = {{coplanar_f_lvids[0], coplanar_f_lvids[1]}};
                if (le[0] > le[1]) std::swap(le[0], le[1]);
                int leid =
                    std::find(local_edges.begin(), local_edges.end(), le) - local_edges.begin();
                is_tet_face_intersected[map_leid2lfids[leid][0]] = true;
                is_tet_face_intersected[map_leid2lfids[leid][1]] = true;
                //
                for (int j = 0; j < 2; j++) {
                    auto conn_tets = m.get_one_ring_tets_for_vertex(vs[coplanar_f_lvids[j]]);
                    for (auto& t : conn_tets) {
                        if (visited.find(t.tid(m)) != visited.end()) continue;
                        visited.insert(t.tid(m));
                        // add lock
                        tet_queue.push(t);
                    }
                }
            }
        } else if (coplanar_f_lvids.size() == 3) {
            bool is_cut = false;
            for (int i = 0; i < 3; i++) {
                std::array<Vector2r, 2> seg2;
                seg2[0] = wmtk::project_point_to_2d(
                    vertex_pos_r(vertex_vids[coplanar_f_lvids[i]]),
                    squeeze_to_2d_dir);
                seg2[1] = wmtk::project_point_to_2d(
                    vertex_pos_r(vertex_vids[coplanar_f_lvids[(i + 1) % 3]]),
                    squeeze_to_2d_dir);
                if (is_seg_cut_tri_2(seg2, tri2)) {
                    is_cut = true;
                    break;
                }
            }
            if (is_cut) {
                std::array<size_t, 3> f = {
                    {vertex_vids[coplanar_f_lvids[0]],
                     vertex_vids[coplanar_f_lvids[1]],
                     vertex_vids[coplanar_f_lvids[2]]}};
                std::sort(f.begin(), f.end());
                tet_face_tags[f].push_back(face_id);
                //
                for (int j = 0; j < 3; j++) {
                    auto conn_tets = m.get_one_ring_tets_for_vertex(vs[coplanar_f_lvids[j]]);
                    for (auto& t : conn_tets) {
                        if (visited.find(t.tid(m)) != visited.end()) continue;
                        visited.insert(t.tid(m));
                        // add lock
                        tet_queue.push(t);
                    }
                }
            }
        }
        //
        if (cnt_pos == 0 || cnt_neg == 0) {
            continue;
        }

        /// check edges
        std::array<Tuple, 6> edges = m.tet_edges(tet);
        //
        std::vector<std::array<size_t, 2>> edge_vids;
        for (auto& loc : edges) {
            size_t v1_id = loc.vid(m);
            auto tmp = m.switch_vertex(loc);
            size_t v2_id = tmp.vid(m);
            std::array<size_t, 2> e = {{v1_id, v2_id}};
            if (e[0] > e[1]) std::swap(e[0], e[1]);
            edge_vids.push_back(e);
        }

        /// check if the tet edges intersects with the triangle
        bool need_subdivision = false;
        for (int l_eid = 0; l_eid < edges.size(); l_eid++) {
            const std::array<size_t, 2>& e = edge_vids[l_eid];
            if (vertex_sides[e[0]] * vertex_sides[e[1]] >= 0) continue;

            if (map_edge2point.count(e)) {
                if (std::get<0>(map_edge2point[e]) == TRI_INTERSECTION) {
                    for (int k = 0; k < 2; k++)
                        is_tet_face_intersected[map_leid2lfids[l_eid][k]] = true;
                    need_subdivision = true;
                }
                continue;
            }

            std::array<Vector3r, 2> seg = {{vertex_pos_r(e[0]), vertex_pos_r(e[1])}};
            Vector3r p(0, 0, 0);
            int intersection_status = EMPTY_INTERSECTION;
            bool is_inside_tri = false;
            bool is_intersected_plane =
                wmtk::open_segment_plane_intersection_3d(seg, tri, p, is_inside_tri);
            if (is_intersected_plane && is_inside_tri) {
                intersection_status = TRI_INTERSECTION;
            } else if (is_intersected_plane) {
                intersection_status = PLN_INTERSECTION;
            }

            map_edge2point[e] = std::make_tuple(intersection_status, p, tet.tid(m), l_eid);
            if (intersection_status == EMPTY_INTERSECTION) {
                continue;
            } else if (intersection_status == TRI_INTERSECTION) {
                for (int k = 0; k < 2; k++)
                    is_tet_face_intersected[map_leid2lfids[l_eid][k]] = true;
                need_subdivision = true;
            }

            // add new tets
            if (need_subdivision) {
                auto incident_tets = m.get_incident_tets_for_edge(edges[l_eid]);
                for (auto& t : incident_tets) {
                    int tid = t.tid(m);
                    if (visited.count(tid)) continue;

                    tet_queue.push(t);
                    visited.insert(tid);
                }
            }
        }

        /// check if the tet (open) face intersects with the triangle
        for (int j = 0; j < 4; j++) { // for each tet face
            if (is_tet_face_intersected[j]) continue;
            std::array<size_t, 3> f = {
                {vs[local_faces[j][0]].vid(m),
                 vs[local_faces[j][1]].vid(m),
                 vs[local_faces[j][2]].vid(m)}};
            std::sort(f.begin(), f.end());
            if (map_face2intersected.count(f)) {
                if (map_face2intersected[f]) need_subdivision = true;
                continue;
            }

            {
                int cnt_pos1 = 0;
                int cnt_neg1 = 0;
                for (int k = 0; k < 3; k++) {
                    if (vertex_sides[f[k]] > 0)
                        cnt_pos1++;
                    else if (vertex_sides[f[k]] < 0)
                        cnt_neg1++;
                }
                if (cnt_pos1 == 0 || cnt_neg1 == 0) continue;
            }

            std::array<Vector3r, 3> tet_tri = {
                {vertex_pos_r(f[0]), vertex_pos_r(f[1]), vertex_pos_r(f[2])}};
            //
            std::array<int, 3> tet_tri_v_sides;
            Vector3r tet_tri_normal = (tet_tri[1] - tet_tri[0]).cross(tet_tri[2] - tet_tri[0]);
            for (int k = 0; k < 3; k++) {
                Vector3r dir = tri[k] - tet_tri[0];
                auto side = dir.dot(tet_tri_normal);
                if (side == 0)
                    tet_tri_v_sides[k] = 0;
                else if (side > 0)
                    tet_tri_v_sides[k] = 1;
                else
                    tet_tri_v_sides[k] = -1;
            }

            bool is_intersected = false;
            for (int k = 0; k < 3; k++) { // check intersection
                if ((tet_tri_v_sides[k] >= 0 && tet_tri_v_sides[(k + 1) % 3] >= 0) ||
                    (tet_tri_v_sides[k] <= 0 && tet_tri_v_sides[(k + 1) % 3] <= 0))
                    continue;
                Vector3r _p;
                is_intersected = wmtk::open_segment_triangle_intersection_3d(
                    {{tri[k], tri[(k + 1) % 3]}},
                    tet_tri,
                    _p);
                if (is_intersected) {
                    need_subdivision = true; // is recorded
                    break;
                }
            }
            map_face2intersected[f] = is_intersected;

            if (is_intersected) {
                auto res = m.switch_tetrahedron(m.tuple_from_face(tet.tid(m), j));
                if (res.has_value()) {
                    auto n_tet = res.value();
                    int tid = n_tet.tid(m);
                    if (visited.find(tid) != visited.end()) continue;
                    // add lock
                    tet_queue.push(n_tet);
                    visited.insert(tid);
                }
            }
        }


        /// record the tets
        if (need_subdivision) {
            intersected_tets.push_back(tet);
            //
            for (auto& e : edge_vids) {
                intersected_tet_edges.insert(e);
            }
        }
    } // End BFS while (!tet_queue.empty())

    // erase edge without intersections OR edge with intersection but not belong to
    // intersected tets
    for (auto it = map_edge2point.begin(), ite = map_edge2point.end(); it != ite;) {
        if (std::get<0>(it->second) == EMPTY_INTERSECTION ||
            !intersected_tet_edges.count(it->first))
            it = map_edge2point.erase(it);
        else
            ++it;
    }
    success_flag = true;
    return std::tuple(success_flag, intersected_tets, map_edge2point);
};

auto internal_triangle_insertion_of_a_queue = [](wmtk::TetMesh& m,
                                                 auto& m_vertex_attribute,
                                                 auto& face_id_cache,
                                                 auto& tet_face_tags,
                                                 const std::vector<Eigen::Vector3d>& vertices,
                                                 const std::vector<std::array<size_t, 3>>& faces,
                                                 auto& Q,
                                                 const auto& check_triangle_acquire,
                                                 const auto& retry_processing,
                                                 const auto& check_edge_acquire,
                                                 const auto& check_tet_acquire,
                                                 const auto& release_locks) {
    using namespace tetwild;
    using Tuple = wmtk::TetMesh::Tuple;

    auto vertex_pos_r = [&m_vertex_attribute](auto i) { return m_vertex_attribute[i].m_pos; };

    std::tuple<double, int, size_t> eiq;

    while (Q.try_pop(eiq)) {
        const auto& [_, retry_time, face_id] = eiq;

        face_id_cache = face_id;

        const auto& [flag, intersected_tets, map_edge2point] = prepare_intersect_info(
            m,
            vertices,
            faces[face_id],
            face_id,
            tet_face_tags,
            check_triangle_acquire,
            check_tet_acquire,
            vertex_pos_r);

        if (!flag) {
            retry_processing(face_id, retry_time);
            continue;
        }
        ///push back new vertices
        std::vector<Tuple> intersected_edges;

        for (auto& info : map_edge2point) {
            auto& [_, p, tid, l_eid] = info.second;
            intersected_edges.push_back(m.tuple_from_edge(tid, l_eid));
        }

        if (check_edge_acquire(intersected_edges) == false ||
            check_tet_acquire(intersected_tets) == false) {
            retry_processing(face_id, retry_time);
            continue;
        }

        std::vector<size_t> new_vids;

        ///inert a triangle
        m.triangle_insertion(intersected_tets, intersected_edges, new_vids);

        assert(new_vids.size() == map_edge2point.size());

        int cnt = 0;
        for (auto& info : map_edge2point) {
            auto& [_, p, tid, l_eid] = info.second;
            m_vertex_attribute[new_vids[cnt]] = VertexAttributes(p);
            cnt++;
        }

        /// Lock specific
        release_locks();
    }
};

void tetwild::TetWild::insert_input_surface(const InputSurface& _input_surface)
{
    const auto& vertices = _input_surface.vertices;
    const auto& faces = _input_surface.faces;
    const auto& partition_id = _input_surface.partition_id;
    triangle_insertion_helper.input_vertices = vertices;
    triangle_insertion_helper.input_faces = faces;

    init_from_delaunay_box_mesh(vertices);

    // match faces preserved in delaunay
    tbb::concurrent_vector<bool> is_matched;
    match_insertion_faces(vertices, faces, is_matched);
    wmtk::logger().info("is_matched: {}", std::count(is_matched.begin(), is_matched.end(), true));

    std::vector<tbb::concurrent_priority_queue<std::tuple<double, int, size_t>>> insertion_queues(
        NUM_THREADS);
    tbb::concurrent_priority_queue<std::tuple<double, int, size_t>> expired_queue;
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, 100.0);
    for (size_t face_id = 0; face_id < faces.size(); face_id++) {
        if (is_matched[face_id]) continue;
        double rand = distribution(generator);
        insertion_queues[partition_id[faces[face_id][0]]].emplace(rand, 0, face_id);
    }

    for (int i = 0; i < NUM_THREADS; i++) {
        wmtk::logger().debug("{}: {}", i, insertion_queues[i].size());
    }

    tbb::task_arena arena(NUM_THREADS);
    tbb::task_group tg;

    arena.execute([&insertion_queues, &tg, &expired_queue, &m = *this, &vertices, &faces]() {
        for (int task_id = 0; task_id < m.NUM_THREADS; task_id++) {
            tg.run([&insertion_queues, &expired_queue, &m, &vertices, &faces, task_id] {
                auto check_tet_acquire = [&m, task_id](const auto& intersected_tets) {
                    for (auto t_int : intersected_tets) {
                        for (auto v_int : m.oriented_tet_vertices(t_int)) {
                            if (!m.try_set_vertex_mutex_one_ring(v_int, task_id)) {
                                return false;
                            }
                        }
                    }
                    return true;
                };

                auto check_edge_acquire = [&m, task_id](const auto& intersected_edges) {
                    for (auto e_int : intersected_edges) {
                        if (!m.try_set_vertex_mutex_one_ring(e_int, task_id)) {
                            return false;
                        }
                        if (!m.try_set_vertex_mutex_one_ring(e_int.switch_vertex(m), task_id)) {
                            return false;
                        }
                    }
                    return true;
                };
                auto release_locks = [&m]() { m.release_vertex_mutex_in_stack(); };

                std::default_random_engine generator;
                std::uniform_real_distribution<double> distribution(0.0, 100.0);
                auto retry_processing = [&,
                                         &Q = insertion_queues[task_id]](auto id, auto retry_time) {
                    double rand = distribution(generator);
                    if (retry_time < 5) {
                        Q.push(std::make_tuple(rand, retry_time + 1, id));
                    } else {
                        expired_queue.push(std::make_tuple(rand, 0, id));
                    }
                };
                auto check_triangle_acquire = [&m, task_id](const auto& f) {
                    return m.try_set_face_mutex_two_ring(f[0], f[1], f[2], task_id);
                };

                internal_triangle_insertion_of_a_queue(
                    m,
                    m.m_vertex_attribute,
                    m.triangle_insertion_local_cache.local().face_id,
                    m.triangle_insertion_helper.tet_face_tags,
                    vertices,
                    faces,
                    insertion_queues[task_id],
                    check_triangle_acquire,
                    retry_processing,
                    check_edge_acquire,
                    check_tet_acquire,
                    release_locks);
            });
        }
    });
    arena.execute([&] { tg.wait(); });

    wmtk::logger().info("expired size: {}", expired_queue.size());

    auto check_acquire = [](const auto&) { return true; };
    auto retry_processing = [](auto, auto) {};

    internal_triangle_insertion_of_a_queue(
        *this,
        m_vertex_attribute,
        triangle_insertion_local_cache.local().face_id,
        triangle_insertion_helper.tet_face_tags,
        vertices,
        faces,
        expired_queue,
        check_acquire,
        retry_processing,
        check_acquire,
        check_acquire,
        []() {});


    //// track surface, bbox, rounding
    wmtk::logger().info("finished insertion");

    setup_attributes();

    wmtk::logger().info("#t {}", tet_capacity());
    wmtk::logger().info("#v {}", vert_capacity());
} // note: skip preserve open boundaries

void tetwild::TetWild::setup_attributes()
{
    auto output_surface = [&](std::string file) {
        std::ofstream fout(file);
        std::vector<std::array<int, 3>> fs;
        int cnt = 0;

        for (auto& info : triangle_insertion_helper.tet_face_tags) {
            auto& vids = info.first;
            auto& fids = info.second;

            if (fids.empty()) continue;

            fout << "v " << m_vertex_attribute[vids[0]].m_posf.transpose() << std::endl;
            fout << "v " << m_vertex_attribute[vids[1]].m_posf.transpose() << std::endl;
            fout << "v " << m_vertex_attribute[vids[2]].m_posf.transpose() << std::endl;
            fs.push_back({{cnt * 3 + 1, cnt * 3 + 2, cnt * 3 + 3}});
            cnt++;
        }

        for (auto& f : fs) fout << "f " << f[0] << " " << f[1] << " " << f[2] << std::endl;
        fout.close();
    };

    const auto& vertices = triangle_insertion_helper.input_vertices;
    const auto& faces = triangle_insertion_helper.input_faces;

    tbb::task_arena arena(NUM_THREADS);

    arena.execute([&vertices, &faces, this] {
        tbb::parallel_for(
            triangle_insertion_helper.tet_face_tags.range(),
            [&vertices, &faces, this](
                tbb::concurrent_map<std::array<size_t, 3>, std::vector<int>>::const_range_type& r) {
                for (tbb::concurrent_map<std::array<size_t, 3>, std::vector<int>>::const_iterator
                         i = r.begin();
                     i != r.end();
                     i++) {
                    auto& info = i;
                    auto& vids = info->first;
                    auto fids = info->second;
                    if (fids.empty()) continue;

                    Vector3r c = m_vertex_attribute[vids[0]].m_pos +
                                 m_vertex_attribute[vids[1]].m_pos +
                                 m_vertex_attribute[vids[2]].m_pos;
                    c = c / 3;

                    wmtk::vector_unique(fids);

                    int inside_fid = -1;
                    for (int input_fid : fids) {
                        std::array<Vector3r, 3> tri = {
                            {to_rational(vertices[faces[input_fid][0]]),
                             to_rational(vertices[faces[input_fid][1]]),
                             to_rational(vertices[faces[input_fid][2]])}};
                        //
                        std::array<Vector2r, 3> tri2;
                        int squeeze_to_2d_dir = wmtk::project_triangle_to_2d(tri, tri2);
                        auto c2 = wmtk::project_point_to_2d(c, squeeze_to_2d_dir);
                        //
                        if (wmtk::is_point_inside_triangle(
                                c2,
                                tri2)) { // should exclude the points on the edges of tri2 -- NO
                            auto [face, global_tet_fid] = tuple_from_face(vids);
                            m_face_attribute[global_tet_fid].m_is_surface_fs = 1;
                            //
                            for (size_t vid : vids) {
                                m_vertex_attribute[vid].m_is_on_surface = true;
                            }
                            //
                            inside_fid = input_fid;
                            break;
                        }
                    }
                }
            });

        //// track bbox
        auto faces = get_faces();

        for (int i = 0; i < faces.size(); i++) {
            auto vs = get_face_vertices(faces[i]);
            std::array<size_t, 3> vids = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
            int on_bbox = -1;
            for (int k = 0; k < 3; k++) {
                if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_min[k] &&
                    m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_min[k] &&
                    m_vertex_attribute[vids[2]].m_pos[k] == m_params.box_min[k]) {
                    on_bbox = k * 2;
                    break;
                }
                if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_max[k] &&
                    m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_max[k] &&
                    m_vertex_attribute[vids[2]].m_pos[k] == m_params.box_max[k]) {
                    on_bbox = k * 2 + 1;
                    break;
                }
            }
            if (on_bbox < 0) continue;
            auto fid = faces[i].fid(*this);
            m_face_attribute[fid].m_is_bbox_fs = on_bbox;
            //
            for (size_t vid : vids) {
                m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
            }
        }


        tbb::parallel_for(
            tbb::blocked_range<int>(0, m_vertex_attribute.m_attributes.size()),
            [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++)
                    wmtk::vector_unique(m_vertex_attribute[i].on_bbox_faces);
            });

        //// rounding
        std::atomic_int cnt_round(0);

        for (int i = 0; i < m_vertex_attribute.m_attributes.size(); i++) {
            auto v = tuple_from_vertex(i);
            if (round(v)) cnt_round++;
        }


        wmtk::logger().info("cnt_round {}/{}", cnt_round, m_vertex_attribute.m_attributes.size());

        //// init qualities

        tbb::parallel_for(
            tbb::blocked_range<int>(0, m_tet_attribute.size()),
            [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    m_tet_attribute[i].m_quality = get_quality(tuple_from_tet(i));
                }
            });
    });
}

void tetwild::TetWild::add_tet_centroid(const Tuple& t, size_t vid)
{
    auto vs = oriented_tet_vertices(t);

    m_vertex_attribute[vid] = VertexAttributes(
        (m_vertex_attribute[vs[0].vid(*this)].m_pos + m_vertex_attribute[vs[1].vid(*this)].m_pos +
         m_vertex_attribute[vs[2].vid(*this)].m_pos + m_vertex_attribute[vs[3].vid(*this)].m_pos) /
        4);
}