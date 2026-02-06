#pragma once

#include "wmtk/TetMesh.h"
#include "wmtk/utils/GeoUtils.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/concurrent_map.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <Eigen/Core>
#include <array>
#include <vector>
#include <wmtk/Types.hpp>

namespace wmtk {
/**
 * @brief Before triangle insertion, find which ones are already present in the mesh.
 * Note that the vertices are already the same, so just do a dictionary-find for the face
 * indices.
 * @param faces
 * @param output is_matched
 */
void match_tet_faces_to_triangles(
    const wmtk::TetMesh& m,
    const std::vector<std::array<size_t, 3>>& faces,
    tbb::concurrent_vector<bool>& is_matched,
    tbb::concurrent_map<std::array<size_t, 3>, std::vector<int>>& tet_face_tags);

bool remove_duplicates(
    std::vector<Vector3d>& vertices,
    std::vector<std::array<size_t, 3>>& faces,
    const double epsilon);

template <typename rational>
auto triangle_insert_prepare_info(
    const wmtk::TetMesh& m,
    const std::array<size_t, 3>& face_v,
    std::vector<std::array<size_t, 3>>& marking_tet_faces,
    const std::function<bool(const std::array<size_t, 3>&)>& try_acquire_triangle,
    const std::function<bool(const std::vector<wmtk::TetMesh::Tuple>&)>& try_acquire_tetra,
    const std::function<Eigen::Matrix<rational, 3, 1>(size_t)>& vertex_pos_r)
{
    using Vector3r = Eigen::Matrix<rational, 3, 1>;
    using Vector2r = Eigen::Matrix<rational, 2, 1>;
    using Tuple = wmtk::TetMesh::Tuple;
    constexpr int EMPTY_INTERSECTION = 0;
    constexpr int TRI_INTERSECTION = 1;
    constexpr int PLN_INTERSECTION = 2;

    static constexpr std::array<std::array<int, 2>, 6> map_leid2lfids = {
        {{{0, 2}}, {{0, 3}}, {{0, 1}}, {{1, 2}}, {{2, 3}}, {{1, 3}}}};
    static constexpr std::array<std::array<int, 3>, 4> local_faces = {
        {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 1, 3}}, {{1, 2, 3}}}};
    static constexpr std::array<std::array<int, 2>, 6> local_edges = {
        {{{0, 1}}, {{1, 2}}, {{0, 2}}, {{0, 3}}, {{1, 3}}, {{2, 3}}}};

    bool success_flag = false;
    std::set<size_t> visited;

    std::array<Vector3r, 3> tri = {
        vertex_pos_r(face_v[0]),
        vertex_pos_r(face_v[1]),
        vertex_pos_r(face_v[2])};
    //
    Vector3r tri_normal = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
    //
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

    if (!try_acquire_triangle(face_v)) {
        return std::tuple(
            success_flag,
            std::vector<Tuple>(),
            std::vector<Tuple>(),
            std::vector<Vector3r>());
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
    constexpr auto is_seg_cut_tri_2 = [](const std::array<Vector2r, 2>& seg2d,
                                         const std::array<Vector2r, 3>& tri2d) {
        // overlap == seg has one endpoint inside tri OR seg intersect with tri edges
        bool is_inside = wmtk::is_point_inside_triangle(seg2d[0], tri2d) ||
                         wmtk::is_point_inside_triangle(seg2d[1], tri2d);
        if (is_inside) {
            return true;
        } else {
            for (int j = 0; j < 3; j++) {
                std::array<Vector2r, 2> tri_seg2 = {{tri2d[j], tri2d[(j + 1) % 3]}};
                rational _;
                bool is_intersected =
                    wmtk::open_segment_open_segment_intersection_2d(seg2d, tri_seg2, _);
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

        auto retry_flag = !try_acquire_tetra(std::vector<Tuple>{{tet}});
        if (retry_flag) {
            return std::tuple(
                success_flag,
                std::vector<Tuple>(),
                std::vector<Tuple>(),
                std::vector<Vector3r>());
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
            size_t vid = vertex_vids[lvid];
            auto p = wmtk::project_point_to_2d(vertex_pos_r(vid), squeeze_to_2d_dir);
            bool is_inside = wmtk::is_point_inside_triangle(p, tri2);
            //
            if (is_inside) {
                auto conn_tets = m.get_one_ring_tets_for_vertex(vs[lvid]);
                for (auto& t : conn_tets) {
                    if (visited.find(t.tid(m)) != visited.end()) continue;
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
                marking_tet_faces.push_back(f);
                // tet_face_tags[f].push_back(face_id);
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

            if (map_edge2point.find(e) != map_edge2point.end()) {
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
                for (const auto& t : incident_tets) {
                    size_t tid = t.tid(m);
                    if (visited.find(tid) != visited.end()) {
                        continue;
                    }

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
            auto it = map_face2intersected.find(f);
            if (it != map_face2intersected.end()) {
                if (it->second) need_subdivision = true;
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
                    size_t tid = n_tet.tid(m);
                    if (visited.find(tid) != visited.end()) {
                        continue;
                    }
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
            intersected_tet_edges.find(it->first) == intersected_tet_edges.end())
            it = map_edge2point.erase(it);
        else
            ++it;
    }
    success_flag = true;
    std::vector<TetMesh::Tuple> intersected_edges;
    std::vector<Vector3r> intersected_pos;
    for (auto& info : map_edge2point) {
        auto& [_, p, tid, l_eid] = info.second;
        intersected_edges.push_back(m.tuple_from_edge(tid, l_eid));
        intersected_pos.push_back(p);
    }
    return std::tuple(success_flag, intersected_tets, intersected_edges, intersected_pos);
}
} // namespace wmtk