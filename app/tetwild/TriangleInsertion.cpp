#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>
#include <atomic>
#include <random>
#include <wmtk/utils/Delaunay.hpp>
#include "TetWild.h"
#include "wmtk/TetMesh.h"
#include "wmtk/auto_table.hpp"
#include "wmtk/utils/GeoUtils.h"
#include "wmtk/utils/Logger.hpp"

#include <igl/remove_duplicate_vertices.h>

#include <fstream>
#include <unordered_set>

bool tetwild::TetWild::InputSurface::remove_duplicates()
{
    Eigen::MatrixXd V_tmp(vertices.size(), 3), V_in;
    Eigen::MatrixXi F_tmp(faces.size(), 3), F_in;
    for (int i = 0; i < vertices.size(); i++) V_tmp.row(i) = vertices[i];
    for (int i = 0; i < faces.size(); i++)
        F_tmp.row(i) << faces[i][0], faces[i][1], faces[i][2]; // note: using int here

    //
    Eigen::VectorXi IV, _;
    igl::remove_duplicate_vertices(V_tmp, F_tmp, SCALAR_ZERO * params.diag_l, V_in, IV, _, F_in);
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
        if (area.norm() / 2 <= SCALAR_ZERO * params.diag_l) continue;
        out_faces.push_back({{(size_t)F_in(i, 0), (size_t)F_in(i, 1), (size_t)F_in(i, 2)}});
        //        input_tags.push_back(old_input_tags[i]);
    }

    vertices = out_vertices;
    faces = out_faces;

    return true;
}

void tetwild::TetWild::construct_background_mesh(const InputSurface& input_surface)
{
    const auto& vertices = input_surface.vertices;
    const auto& faces = input_surface.faces;

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
    // todo: track bbox

    //    output_mesh("delaunay.msh");
}

void tetwild::TetWild::match_insertion_faces(
    const InputSurface& input_surface,
    tbb::concurrent_vector<bool>& is_matched)
{
    const auto& vertices = input_surface.vertices;
    const auto& faces = input_surface.faces;
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
            if (map_surface.count(f)) {
                int fid = map_surface[f];
                triangle_insertion_global_cache.tet_face_tags[f].push_back(fid);
                is_matched[fid] = true;
            }
        }
    }
}

void tetwild::TetWild::triangle_insertion_before(const std::vector<Tuple>& faces)
{
    triangle_insertion_local_cache.local().old_face_vids.clear(); // note: reset local vars

    for (auto& loc : faces) {
        auto vs = get_face_vertices(loc);
        std::array<size_t, 3> f = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
        std::sort(f.begin(), f.end());

        triangle_insertion_local_cache.local().old_face_vids.push_back(f);
    }
}

void tetwild::TetWild::triangle_insertion_after(
    const std::vector<Tuple>& old_faces,
    const std::vector<std::vector<Tuple>>& new_faces)
{
    auto& tet_face_tags = triangle_insertion_global_cache.tet_face_tags;

    /// remove old_face_vids from tet_face_tags, and map tags to new faces
    // assert(new_faces.size() == triangle_insertion_local_cache.local().old_face_vids.size() + 1);

    for (int i = 0; i < new_faces.size(); i++) {
        if (new_faces[i].empty()) continue;

        // note: erase old tag and then add new -- old and new can be the same face
        std::vector<int> tags;
        if (i < triangle_insertion_local_cache.local().old_face_vids.size()) {
            auto& old_f = triangle_insertion_local_cache.local().old_face_vids[i];
            if (tet_face_tags.count(old_f) && !tet_face_tags[old_f].empty()) {
                tags = tet_face_tags[old_f];
                //                tet_face_tags.unsafe_erase(triangle_insertion_local_cache.local().old_face_vids[i]);
                //                tet_face_tags.unsafe_erase(old_f);
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
}

void tetwild::TetWild::triangle_insertion_stuff(
    std::vector<tbb::concurrent_priority_queue<std::tuple<double, int, size_t>>>& insertion_queues,
    tbb::concurrent_queue<size_t>& expired_queue,
    int task_id)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, 100.0);

    constexpr int EMPTY_INTERSECTION = 0;
    constexpr int TRI_INTERSECTION = 1;
    constexpr int PLN_INTERSECTION = 2;

    static constexpr std::array<std::array<int, 2>, 6> map_leid2lfids = {
        {{{0, 2}}, {{0, 3}}, {{0, 1}}, {{1, 2}}, {{2, 3}}, {{1, 3}}}};
    static constexpr std::array<std::array<int, 3>, 4> map_lvid2lfids = {
        {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 1, 3}}, {{1, 2, 3}}}};
    static const std::array<std::array<int, 3>, 4> local_faces = {
        {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 1, 3}}, {{1, 2, 3}}}};
    static const std::array<std::array<int, 2>, 6> local_edges = {
        {{{0, 1}}, {{1, 2}}, {{0, 2}}, {{0, 3}}, {{1, 3}}, {{2, 3}}}};

    std::tuple<double, int, size_t> eiq;

    auto& is_matched = triangle_insertion_global_cache.is_matched;
    //    auto& is_visited = triangle_insertion_local_cache.local().is_visited;
    const auto& vertices = triangle_insertion_global_cache.input_surface.vertices;
    const auto& faces = triangle_insertion_global_cache.input_surface.faces;

    while (insertion_queues[task_id].try_pop(eiq)) {
        auto [prio, retry_time, face_id] = eiq;

        if (is_matched[face_id]) continue;

        triangle_insertion_local_cache.local().face_id = face_id;
        //        is_visited.assign(m_tet_attribute.size(), false); // reset
        std::set<size_t> visited;

        std::array<Vector3r, 3> tri = {
            {to_rational(vertices[faces[face_id][0]]),
             to_rational(vertices[faces[face_id][1]]),
             to_rational(vertices[faces[face_id][2]])}};
        std::array<Vector3d, 3> tri_d = {
            {vertices[faces[face_id][0]],
             vertices[faces[face_id][1]],
             vertices[faces[face_id][2]]}};
        //
        Vector3r tri_normal = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
        //
        Vector3d tri_normal_d = (tri_d[1] - tri_d[0]).cross(tri_d[2] - tri_d[0]);
        std::array<Vector2r, 3> tri2;
        int squeeze_to_2d_dir = wmtk::project_triangle_to_2d(tri, tri2);

        std::vector<Tuple> intersected_tets;
        std::map<std::array<size_t, 2>, std::tuple<int, Vector3r, size_t, int>> map_edge2point;
        std::map<std::array<size_t, 3>, bool> map_face2intersected;
        // e = (e0, e1) ==> (intersection_status, intersection_point, edge's_tid, local_eid_in_tet)
        std::set<std::array<size_t, 2>> intersected_tet_edges;
        //
        std::queue<Tuple> tet_queue;
        //

        // set_locks
        std::vector<size_t> mutex_release_stack;

        auto v1 = faces[face_id][0];
        auto v2 = faces[face_id][1];
        auto v3 = faces[face_id][2];

        if (!try_set_face_mutex_two_ring(v1, v2, v3, task_id)) {
            // retry
            if (retry_time < 5) {
                double rand = distribution(generator);
                insertion_queues[task_id].push(std::make_tuple(rand, retry_time + 1, face_id));
            } else {
                expired_queue.push(face_id);
            }

            continue;
        }

        for (int j = 0; j < 3; j++) {
            Tuple loc = tuple_from_vertex(faces[face_id][j]);
            auto conn_tets = get_one_ring_tets_for_vertex(loc);
            for (const auto& t : conn_tets) {
                //                if (is_visited[t.tid(*this)]) continue;
                if (visited.count(t.tid(*this))) continue;
                tet_queue.push(t);
                //                is_visited[t.tid(*this)] = true;
                visited.insert(t.tid(*this));
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

        bool retry_flag = false;
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
            auto vs = oriented_tet_vertices(tet);


            for (auto v_id : vs) {
                if (!try_set_vertex_mutex_one_ring(v_id, task_id)) {
                    retry_flag = true;
                    break;
                }
            }

            if (retry_flag) break;
            // add lock

            std::array<size_t, 4> vertex_vids;
            for (int j = 0; j < 4; j++) {
                vertex_vids[j] = vs[j].vid(*this);
                Vector3r dir = m_vertex_attribute[vertex_vids[j]].m_pos - tri[0];
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
                Vector2r p =
                    wmtk::project_point_to_2d(m_vertex_attribute[vid].m_pos, squeeze_to_2d_dir);
                bool is_inside = wmtk::is_point_inside_triangle(p, tri2);
                //
                if (is_inside) {
                    //                    for (int j = 0; j < 3; j++)
                    //                        is_tet_face_intersected[map_lvid2lfids[lvid][j]] =
                    //                        true;
                    // note: cannot add this ^, since you are using open segments
                    auto conn_tets = get_one_ring_tets_for_vertex(vs[lvid]);
                    for (auto& t : conn_tets) {
                        if (visited.count(t.tid(*this))) continue;
                        visited.insert(t.tid(*this));
                        tet_queue.push(t);
                        // add lock
                    }
                }
            } else if (coplanar_f_lvids.size() == 2) {
                std::array<Vector2r, 2> seg2;
                seg2[0] = wmtk::project_point_to_2d(
                    m_vertex_attribute[vertex_vids[coplanar_f_lvids[0]]].m_pos,
                    squeeze_to_2d_dir);
                seg2[1] = wmtk::project_point_to_2d(
                    m_vertex_attribute[vertex_vids[coplanar_f_lvids[1]]].m_pos,
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
                        auto conn_tets = get_one_ring_tets_for_vertex(vs[coplanar_f_lvids[j]]);
                        for (auto& t : conn_tets) {
                            if (visited.count(t.tid(*this))) continue;
                            visited.insert(t.tid(*this));
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
                        m_vertex_attribute[vertex_vids[coplanar_f_lvids[i]]].m_pos,
                        squeeze_to_2d_dir);
                    seg2[1] = wmtk::project_point_to_2d(
                        m_vertex_attribute[vertex_vids[coplanar_f_lvids[(i + 1) % 3]]].m_pos,
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
                    triangle_insertion_global_cache.tet_face_tags[f].push_back(face_id);
                    //
                    for (int j = 0; j < 3; j++) {
                        auto conn_tets = get_one_ring_tets_for_vertex(vs[coplanar_f_lvids[j]]);
                        for (auto& t : conn_tets) {
                            //                            if (is_visited[t.tid(*this)]) continue;
                            //                            is_visited[t.tid(*this)] = true;
                            if (visited.count(t.tid(*this))) continue;
                            visited.insert(t.tid(*this));
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
            std::array<Tuple, 6> edges = tet_edges(tet);
            //
            std::vector<std::array<size_t, 2>> edge_vids;
            for (auto& loc : edges) {
                size_t v1_id = loc.vid(*this);
                auto tmp = switch_vertex(loc);
                size_t v2_id = tmp.vid(*this);
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

                std::array<Vector3r, 2> seg = {
                    {m_vertex_attribute[e[0]].m_pos, m_vertex_attribute[e[1]].m_pos}};
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

                map_edge2point[e] = std::make_tuple(intersection_status, p, tet.tid(*this), l_eid);
                if (intersection_status == EMPTY_INTERSECTION) {
                    continue;
                } else if (intersection_status == TRI_INTERSECTION) {
                    for (int k = 0; k < 2; k++)
                        is_tet_face_intersected[map_leid2lfids[l_eid][k]] = true;
                    need_subdivision = true;
                }

                // add new tets
                if (need_subdivision) {
                    auto incident_tets = get_incident_tets_for_edge(edges[l_eid]);
                    for (auto& t : incident_tets) {
                        int tid = t.tid(*this);
                        //                        if (is_visited[tid]) continue;
                        if (visited.count(tid)) continue;

                        //\add lock
                        tet_queue.push(t);
                        //                        is_visited[tid] = true;
                        visited.insert(tid);
                    }
                }
            }

            /// check if the tet (open) face intersects with the triangle
            //            if (!need_subdivision) {
            //            if (std::count(is_tet_face_intersected.begin(),
            //            is_tet_face_intersected.end(), true)) { std::array<Tuple, 4> fs; fs[0] =
            //            tet; fs[1] = switch_face(tet); auto tet1 =
            //            switch_edge(switch_vertex(tet)); fs[2] = switch_face(tet1); auto tet2 =
            //            switch_edge(switch_vertex(tet1)); fs[3] = switch_face(tet2);
            for (int j = 0; j < 4; j++) { // for each tet face
                if (is_tet_face_intersected[j]) continue;
                //                auto fvs = get_face_vertices(fs[j]);
                //                std::array<size_t, 3> f = {
                //                    {fvs[0].vid(*this), fvs[1].vid(*this), fvs[2].vid(*this)}};
                std::array<size_t, 3> f = {
                    {vs[local_faces[j][0]].vid(*this),
                     vs[local_faces[j][1]].vid(*this),
                     vs[local_faces[j][2]].vid(*this)}};
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
                    {m_vertex_attribute[f[0]].m_pos,
                     m_vertex_attribute[f[1]].m_pos,
                     m_vertex_attribute[f[2]].m_pos}};
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
                    //                    auto res = switch_tetrahedron(fs[j]);
                    auto res = switch_tetrahedron(tuple_from_face(tet.tid(*this), j));
                    if (res.has_value()) {
                        auto n_tet = res.value();
                        int tid = n_tet.tid(*this);
                        //                            if (is_visited[tid]) continue;
                        if (visited.count(tid)) continue;
                        // add lock
                        tet_queue.push(n_tet);
                        //                            is_visited[tid] = true;
                        visited.insert(tid);
                    }
                }
            }
            //            }


            /// record the tets
            if (need_subdivision) {
                intersected_tets.push_back(tet);
                //
                for (auto& e : edge_vids) {
                    intersected_tet_edges.insert(e);
                }
            }
        }

        if (retry_flag) {
            if (retry_time < 5) {
                double rand = distribution(generator);
                insertion_queues[task_id].push(std::make_tuple(rand, retry_time + 1, face_id));
            } else {
                expired_queue.push(face_id);
            }
            continue;
        }

        // erase edge without intersections OR edge with intersection but not belong to intersected
        // tets
        for (auto it = map_edge2point.begin(), ite = map_edge2point.end(); it != ite;) {
            if (std::get<0>(it->second) == EMPTY_INTERSECTION ||
                !intersected_tet_edges.count(it->first))
                it = map_edge2point.erase(it);
            else
                ++it;
        }

        ///push back new vertices
        std::vector<Tuple> intersected_edges;
        // std::map<std::array<size_t, 2>, size_t> map_edge2vid;


        for (auto& info : map_edge2point) {
            auto& [_, p, tid, l_eid] = info.second;
            VertexAttributes v;
            v.m_pos = p;
            v.m_posf = to_double(v.m_pos);
            //            m_vertex_attribute.m_attributes.push_back(v);
            //            (info.second).first = m_vertex_attribute.m_attributes.size() - 1;//todo: check if needed

            ///
            intersected_edges.push_back(tuple_from_edge(tid, l_eid));
        }

        // set locks for intersected edges and tets
        // std::vector<size_t> mutex_release_stack;
        bool continue_flag = false;
        for (auto e_int : intersected_edges) {
            if (!try_set_vertex_mutex_one_ring(e_int, task_id)) {
                continue_flag = true;
                break;
            }
            if (!try_set_vertex_mutex_one_ring(e_int.switch_vertex(*this), task_id)) {
                continue_flag = true;
                break;
            }
        }

        if (continue_flag) continue;

        for (auto t_int : intersected_tets) {
            for (auto v_int : oriented_tet_vertices(t_int)) {
                if (!try_set_vertex_mutex_one_ring(v_int, task_id)) {
                    continue_flag = true;
                    break;
                }
            }
            if (continue_flag) {
                break;
            }
        }

        if (continue_flag) {
            if (retry_time < 5) {
                double rand = distribution(generator);
                insertion_queues[task_id].push(std::make_tuple(rand, retry_time + 1, face_id));
            } else {
                expired_queue.push(face_id);
            }
            continue;
        }

        std::vector<size_t> new_vids;
        std::vector<size_t> new_tids;
        std::vector<size_t> new_center_vids;

        ///inert a triangle
        single_triangle_insertion(
            intersected_tets,
            intersected_edges,
            new_vids,
            new_tids,
            new_center_vids);


        assert(new_vids.size() == map_edge2point.size());

        int cnt = 0;
        for (auto& info : map_edge2point) {
            auto& [_, p, tid, l_eid] = info.second;
            VertexAttributes v;
            v.m_pos = p;
            v.m_posf = to_double(v.m_pos);
            m_vertex_attribute[new_vids[cnt]] = v;
            cnt++;
        }

        int num_released = release_vertex_mutex_in_stack();
    }
}

void tetwild::TetWild::triangle_insertion(const InputSurface& _input_surface)
{
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, 100.0);
    constexpr int EMPTY_INTERSECTION = 0;
    constexpr int TRI_INTERSECTION = 1;
    constexpr int PLN_INTERSECTION = 2;

    static constexpr std::array<std::array<int, 2>, 6> map_leid2lfids = {
        {{{0, 2}}, {{0, 3}}, {{0, 1}}, {{1, 2}}, {{2, 3}}, {{1, 3}}}};
    static constexpr std::array<std::array<int, 3>, 6> map_lvid2lfids = {
        {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 1, 3}}, {{1, 2, 3}}}};
    static constexpr std::array<std::array<int, 3>, 4> local_faces = {
        {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 1, 3}}, {{1, 2, 3}}}};
    static constexpr std::array<std::array<int, 2>, 6> local_edges = {
        {{{0, 1}}, {{1, 2}}, {{0, 2}}, {{0, 3}}, {{1, 3}}, {{2, 3}}}};

    triangle_insertion_global_cache.input_surface = _input_surface; // todo: avoid copy
    const auto& input_surface = _input_surface;

    construct_background_mesh(input_surface);
    const auto& vertices = input_surface.vertices;
    const auto& faces = input_surface.faces;

    // fortest
    auto print = [](const Vector3r& p) { wmtk::logger().info("{} {} {}", p[0], p[1], p[2]); };
    auto print2 = [](const Vector2r& p) { wmtk::logger().info("{} {}", p[0], p[1]); };

    // match faces preserved in delaunay
    auto& is_matched = triangle_insertion_global_cache.is_matched;
    match_insertion_faces(input_surface, is_matched);
    wmtk::logger().info("is_matched: {}", std::count(is_matched.begin(), is_matched.end(), true));

    auto& is_visited = triangle_insertion_local_cache.local().is_visited;

    std::vector<tbb::concurrent_priority_queue<std::tuple<double, int, size_t>>> insertion_queues(
        NUM_THREADS);
    tbb::concurrent_queue<size_t> expired_queue;
    for (size_t face_id = 0; face_id < faces.size(); face_id++) {
        double rand = distribution(generator);
        insertion_queues[input_surface.partition_id[faces[face_id][0]]].push(
            std::make_tuple(rand, 0, face_id));
    }

    for (int i = 0; i < NUM_THREADS; i++) {
        std::cout << i << ": " << insertion_queues[i].size() << std::endl;
    }

    tbb::task_arena arena(NUM_THREADS);
    tbb::task_group tg;

    arena.execute([&insertion_queues, &tg, &expired_queue, this]() {
        for (int i = 0; i < this->NUM_THREADS; i++) {
            int j = i;
            tg.run([&insertion_queues, &expired_queue, this, j] {
                triangle_insertion_stuff(insertion_queues, expired_queue, j);
            });
        }
    });
    arena.execute([&] { tg.wait(); });

    // serial dealings

    wmtk::logger().info("expired size: {}", expired_queue.unsafe_size());
    // std::cout<<"expired size: "<<expired_queue.unsafe_size()<<std::endl;

    arena.execute([&expired_queue,
                   &faces,
                   &vertices,
                   &is_matched,
                   this,
                   &tg,
                   TRI_INTERSECTION,
                   EMPTY_INTERSECTION,
                   PLN_INTERSECTION] {
        tg.run([&expired_queue,
                &faces,
                &vertices,
                &is_matched,
                TRI_INTERSECTION,
                EMPTY_INTERSECTION,
                PLN_INTERSECTION,
                this] {
            size_t face_id;
            while (expired_queue.try_pop(face_id)) {
                if (is_matched[face_id]) continue;

                triangle_insertion_local_cache.local().face_id = face_id;
                std::unordered_set<size_t> visited;

                std::array<Vector3r, 3> tri = {
                    {to_rational(vertices[faces[face_id][0]]),
                     to_rational(vertices[faces[face_id][1]]),
                     to_rational(vertices[faces[face_id][2]])}};
                std::array<Vector3d, 3> tri_d = {
                    {vertices[faces[face_id][0]],
                     vertices[faces[face_id][1]],
                     vertices[faces[face_id][2]]}};
                //
                Vector3r tri_normal = (tri[1] - tri[0]).cross(tri[2] - tri[0]);
                //
                Vector3d tri_normal_d = (tri_d[1] - tri_d[0]).cross(tri_d[2] - tri_d[0]);
                std::array<Vector2r, 3> tri2;
                int squeeze_to_2d_dir = wmtk::project_triangle_to_2d(tri, tri2);

                std::vector<Tuple> intersected_tets;
                std::map<std::array<size_t, 2>, std::tuple<int, Vector3r, size_t, int>>
                    map_edge2point;
                std::map<std::array<size_t, 3>, bool> map_face2intersected;
                std::set<std::array<size_t, 2>> intersected_tet_edges;
                //
                std::queue<Tuple> tet_queue;
                //
                for (int j = 0; j < 3; j++) {
                    Tuple loc = tuple_from_vertex(faces[face_id][j]);
                    auto conn_tets = get_one_ring_tets_for_vertex(loc);
                    for (const auto& t : conn_tets) {
                        if (visited.count(t.tid(*this))) continue;
                        tet_queue.push(t);
                        visited.insert(t.tid(*this));
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

                bool retry_flag = false;
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
                    auto vs = oriented_tet_vertices(tet);

                    std::array<size_t, 4> vertex_vids;
                    for (int j = 0; j < 4; j++) {
                        vertex_vids[j] = vs[j].vid(*this);
                        Vector3r dir = m_vertex_attribute[vertex_vids[j]].m_pos - tri[0];
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
                        Vector2r p = wmtk::project_point_to_2d(
                            m_vertex_attribute[vid].m_pos,
                            squeeze_to_2d_dir);
                        bool is_inside = wmtk::is_point_inside_triangle(p, tri2);
                        //
                        if (is_inside) {
                            auto conn_tets = get_one_ring_tets_for_vertex(vs[lvid]);
                            for (auto& t : conn_tets) {
                                if (visited.count(t.tid(*this))) continue;
                                visited.insert(t.tid(*this));
                                tet_queue.push(t);
                            }
                        }
                    } else if (coplanar_f_lvids.size() == 2) {
                        std::array<Vector2r, 2> seg2;
                        seg2[0] = wmtk::project_point_to_2d(
                            m_vertex_attribute[vertex_vids[coplanar_f_lvids[0]]].m_pos,
                            squeeze_to_2d_dir);
                        seg2[1] = wmtk::project_point_to_2d(
                            m_vertex_attribute[vertex_vids[coplanar_f_lvids[1]]].m_pos,
                            squeeze_to_2d_dir);
                        if (is_seg_cut_tri_2(seg2, tri2)) {
                            std::array<int, 2> le = {{coplanar_f_lvids[0], coplanar_f_lvids[1]}};
                            if (le[0] > le[1]) std::swap(le[0], le[1]);
                            int leid = std::find(local_edges.begin(), local_edges.end(), le) -
                                       local_edges.begin();
                            is_tet_face_intersected[map_leid2lfids[leid][0]] = true;
                            is_tet_face_intersected[map_leid2lfids[leid][1]] = true;
                            //
                            for (int j = 0; j < 2; j++) {
                                auto conn_tets =
                                    get_one_ring_tets_for_vertex(vs[coplanar_f_lvids[j]]);
                                for (auto& t : conn_tets) {
                                    if (visited.count(t.tid(*this))) continue;
                                    visited.insert(t.tid(*this));
                                    tet_queue.push(t);
                                }
                            }
                        }
                    } else if (coplanar_f_lvids.size() == 3) {
                        bool is_cut = false;
                        for (int i = 0; i < 3; i++) {
                            std::array<Vector2r, 2> seg2;
                            seg2[0] = wmtk::project_point_to_2d(
                                m_vertex_attribute[vertex_vids[coplanar_f_lvids[i]]].m_pos,
                                squeeze_to_2d_dir);
                            seg2[1] = wmtk::project_point_to_2d(
                                m_vertex_attribute[vertex_vids[coplanar_f_lvids[(i + 1) % 3]]]
                                    .m_pos,
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
                            triangle_insertion_global_cache.tet_face_tags[f].push_back(face_id);
                            //
                            for (int j = 0; j < 3; j++) {
                                auto conn_tets =
                                    get_one_ring_tets_for_vertex(vs[coplanar_f_lvids[j]]);
                                for (auto& t : conn_tets) {
                                    if (visited.count(t.tid(*this))) continue;
                                    visited.insert(t.tid(*this));
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
                    std::array<Tuple, 6> edges = tet_edges(tet);
                    //
                    std::vector<std::array<size_t, 2>> edge_vids;
                    for (auto& loc : edges) {
                        size_t v1_id = loc.vid(*this);
                        auto tmp = switch_vertex(loc);
                        size_t v2_id = tmp.vid(*this);
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

                        std::array<Vector3r, 2> seg = {
                            {m_vertex_attribute[e[0]].m_pos, m_vertex_attribute[e[1]].m_pos}};
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

                        map_edge2point[e] =
                            std::make_tuple(intersection_status, p, tet.tid(*this), l_eid);
                        if (intersection_status == EMPTY_INTERSECTION) {
                            continue;
                        } else if (intersection_status == TRI_INTERSECTION) {
                            for (int k = 0; k < 2; k++)
                                is_tet_face_intersected[map_leid2lfids[l_eid][k]] = true;
                            need_subdivision = true;
                        }

                        // add new tets
                        if (need_subdivision) {
                            auto incident_tets = get_incident_tets_for_edge(edges[l_eid]);
                            for (auto& t : incident_tets) {
                                int tid = t.tid(*this);
                                if (visited.count(tid)) continue;

                                // add lock
                                tet_queue.push(t);
                                visited.insert(tid);
                            }
                        }
                    }

                    /// check if the tet (open) face intersects with the triangle
                    //            if (!need_subdivision) {
                    //            if (true) {
                    //                std::array<Tuple, 4> fs;
                    //                fs[0] = tet;
                    //                fs[1] = switch_face(tet);
                    //                auto tet1 = switch_edge(switch_vertex(tet));
                    //                fs[2] = switch_face(tet1);
                    //                auto tet2 = switch_edge(switch_vertex(tet1));
                    //                fs[3] = switch_face(tet2);

                    for (int j = 0; j < 4; j++) { // for each tet face
                        if (is_tet_face_intersected[j]) continue;
                        //                auto vs = get_face_vertices(fs[j]);
                        //                std::array<size_t, 3> f = {
                        //                    {vs[0].vid(*this), vs[1].vid(*this),
                        //                    vs[2].vid(*this)}};
                        std::array<size_t, 3> f = {
                            {vs[local_faces[j][0]].vid(*this),
                             vs[local_faces[j][1]].vid(*this),
                             vs[local_faces[j][2]].vid(*this)}};
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

                        std::array<Vector3r, 3> tet_tri = {{
                            m_vertex_attribute[f[0]].m_pos,
                            m_vertex_attribute[f[1]].m_pos,
                            m_vertex_attribute[f[2]].m_pos,
                        }};
                        //
                        std::array<int, 3> tet_tri_v_sides;
                        Vector3r tet_tri_normal =
                            (tet_tri[1] - tet_tri[0]).cross(tet_tri[2] - tet_tri[0]);
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
                            Vector3r p;
                            is_intersected = wmtk::open_segment_triangle_intersection_3d(
                                {{tri[k], tri[(k + 1) % 3]}},
                                tet_tri,
                                p);
                            if (is_intersected) {
                                need_subdivision = true; // is recorded
                                break;
                            }
                        }
                        map_face2intersected[f] = is_intersected;

                        if (is_intersected) {
                            //                    auto res = switch_tetrahedron(fs[j]);
                            auto res = switch_tetrahedron(tuple_from_face(tet.tid(*this), j));
                            if (res.has_value()) {
                                auto n_tet = res.value();
                                int tid = n_tet.tid(*this);
                                //                            if (is_visited[tid]) continue;
                                if (visited.count(tid)) continue;
                                // add lock
                                tet_queue.push(n_tet);
                                //                            is_visited[tid] = true;
                                visited.insert(tid);
                            }
                        }
                    }
                    //            }


                    /// record the tets
                    if (need_subdivision) {
                        intersected_tets.push_back(tet);
                        //
                        for (auto& e : edge_vids) {
                            intersected_tet_edges.insert(e);
                        }
                    }
                }

                // erase edge without intersections OR edge with intersection but not belong to
                // intersected tets
                for (auto it = map_edge2point.begin(), ite = map_edge2point.end(); it != ite;) {
                    if (std::get<0>(it->second) == EMPTY_INTERSECTION ||
                        !intersected_tet_edges.count(it->first))
                        it = map_edge2point.erase(it);
                    else
                        ++it;
                }

                ///push back new vertices
                std::vector<Tuple> intersected_edges;


                for (auto& info : map_edge2point) {
                    auto& [_, p, tid, l_eid] = info.second;
                    VertexAttributes v;
                    v.m_pos = p;
                    v.m_posf = to_double(v.m_pos);

                    ///
                    intersected_edges.push_back(tuple_from_edge(tid, l_eid));
                }

                // set locks for intersected edges and tets
                // std::vector<size_t> mutex_release_stack;

                std::vector<size_t> new_vids;
                std::vector<size_t> new_tids;
                std::vector<size_t> new_center_vids;

                ///inert a triangle
                single_triangle_insertion(
                    intersected_tets,
                    intersected_edges,
                    new_vids,
                    new_tids,
                    new_center_vids);


                assert(new_vids.size() == map_edge2point.size());

                int cnt = 0;
                for (auto& info : map_edge2point) {
                    auto& [_, p, tid, l_eid] = info.second;
                    VertexAttributes v;
                    v.m_pos = p;
                    v.m_posf = to_double(v.m_pos);
                    m_vertex_attribute[new_vids[cnt]] = v;
                    cnt++;
                }
            }
        });
    });

    arena.execute([&] { tg.wait(); });

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

        for (auto& info : triangle_insertion_global_cache.tet_face_tags) {
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

    const auto& vertices = triangle_insertion_global_cache.input_surface.vertices;
    const auto& faces = triangle_insertion_global_cache.input_surface.faces;

    tbb::task_arena arena(NUM_THREADS);

    arena.execute([&vertices, &faces, this] {
        tbb::parallel_for(
            triangle_insertion_global_cache.tet_face_tags.range(),
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

                    // fortest
                    if (inside_fid >= 0) {
                        fids = {inside_fid};
                    } else {
                        fids = {};
                    }
                    // fortest
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

    //    // fortest
    //    wmtk::logger().info("output surface...");
    //    output_surface("surface.obj");
    //    // fortest
}

void tetwild::TetWild::add_tet_centroid(const Tuple& t, size_t vid)
{
    auto vs = oriented_tet_vertices(t);
    VertexAttributes v;
    v.m_pos =
        (m_vertex_attribute[vs[0].vid(*this)].m_pos + m_vertex_attribute[vs[1].vid(*this)].m_pos +
         m_vertex_attribute[vs[2].vid(*this)].m_pos + m_vertex_attribute[vs[3].vid(*this)].m_pos) /
        4;
    v.m_posf = to_double(v.m_pos);
    m_vertex_attribute[vid] = v;
}