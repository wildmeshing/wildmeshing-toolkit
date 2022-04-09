#include "Rational.hpp"
#include "TetWild.h"

#include <wmtk/utils/Delaunay.hpp>
#include "common.h"
#include "wmtk/TetMesh.h"
#include "wmtk/auto_table.hpp"
#include "wmtk/utils/GeoUtils.h"
#include "wmtk/utils/InsertTriangleUtils.hpp"
#include "wmtk/utils/Logger.hpp"

#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>

#include <random>
#include <unordered_set>


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

bool tetwild::TetWild::triangle_insertion_after(const std::vector<std::vector<Tuple>>& new_faces)
{
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


auto internal_insert_single_triangle(
    wmtk::TetMesh& m,
    tetwild::TetWild::VertAttCol& m_vertex_attribute,
    const std::vector<Eigen::Vector3d>& vertices,
    const std::array<size_t, 3>& face,
    std::vector<std::array<size_t, 3>>& marked_tet_faces,
    const std::function<bool(const std::array<size_t, 3>&)>& try_acquire_triangle,
    const std::function<bool(const std::vector<wmtk::TetMesh::Tuple>&)>& try_acquire_edge,
    const std::function<bool(const std::vector<wmtk::TetMesh::Tuple>&)>& try_acquire_tetra)
{
    using Tuple = wmtk::TetMesh::Tuple;

    auto vertex_pos_r = [&m_vertex_attribute](size_t i) -> tetwild::Vector3r {
        return m_vertex_attribute[i].m_pos;
    };

    const auto& [flag, intersected_tets, intersected_edges, intersected_pos] =
        wmtk::triangle_insert_prepare_info<apps::Rational>(
            m,
            vertices,
            face,
            marked_tet_faces, // output
            try_acquire_triangle,
            try_acquire_tetra,
            vertex_pos_r);

    if (!flag) {
        return false;
    }

    if (try_acquire_edge(intersected_edges) == false ||
        try_acquire_tetra(intersected_tets) == false) {
        return false;
    }

    // these are only those on edges.
    std::vector<size_t> new_edge_vids;
    std::vector<size_t> new_center_vids;
    std::vector<std::array<size_t, 4>> center_split_tets;

    ///inert a triangle
    m.triangle_insertion(
        intersected_tets,
        intersected_edges,
        new_edge_vids,
        new_center_vids,
        center_split_tets);

    assert(new_center_vids.size() == center_split_tets.size());
    for (auto i = 0; i < new_center_vids.size(); i++) {
        auto vid = new_center_vids[i];
        auto& vs = center_split_tets[i];
        m_vertex_attribute[vid] = tetwild::VertexAttributes(
            (m_vertex_attribute[vs[0]].m_pos + m_vertex_attribute[vs[1]].m_pos +
             m_vertex_attribute[vs[2]].m_pos + m_vertex_attribute[vs[3]].m_pos) /
            4);
    }
    assert(new_edge_vids.size() == intersected_pos.size());

    for (auto i=0; i<intersected_pos.size(); i++) {
        m_vertex_attribute[new_edge_vids[i]] = tetwild::VertexAttributes(intersected_pos[i]);
    }

    return true;
};

void tetwild::TetWild::init_from_input_surface(
    const std::vector<Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces,
    const std::vector<size_t>& partition_id)
{
    init_from_delaunay_box_mesh(vertices);

    // match faces preserved in delaunay
    tbb::concurrent_vector<bool> is_matched;
    wmtk::match_tet_faces_to_triangles(*this, vertices, faces, is_matched, tet_face_tags);
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

    arena.execute([&insertion_queues,
                   &tg,
                   &expired_queue,
                   &m = *this,
                   &tet_face_tags = this->tet_face_tags,
                   &vertices,
                   &faces]() {
        for (int task_id = 0; task_id < m.NUM_THREADS; task_id++) {
            tg.run([&insertion_queues,
                    &expired_queue,
                    &tet_face_tags,
                    &m,
                    &vertices,
                    &faces,
                    task_id] {
                auto try_acquire_tetra = [&m, task_id](const auto& intersected_tets) {
                    for (auto t_int : intersected_tets) {
                        for (auto v_int : m.oriented_tet_vertices(t_int)) {
                            if (!m.try_set_vertex_mutex_one_ring(v_int, task_id)) {
                                return false;
                            }
                        }
                    }
                    return true;
                };

                auto try_acquire_edge = [&m, task_id](const auto& intersected_edges) {
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
                auto try_acquire_triangle = [&m, task_id](const auto& f) {
                    return m.try_set_face_mutex_two_ring(f[0], f[1], f[2], task_id);
                };

                auto supply_element = [&m,
                                       &Q = insertion_queues[task_id],
                                       &face_id_cache =
                                           m.triangle_insertion_local_cache.local().face_id,
                                       &retry_processing](const auto& func) {
                    std::tuple<double, int, size_t> eiq;
                    while (Q.try_pop(eiq)) {
                        const auto& [_, retry_time, face_id] = eiq;

                        face_id_cache = face_id;
                        if (func(face_id) == false) {
                            retry_processing(face_id, retry_time);
                            continue;
                        };
                        m.release_vertex_mutex_in_stack();
                    }
                };

                supply_element([&](auto face_id) {
                    std::vector<std::array<size_t, 3>> marked_tet_faces;
                    auto success = internal_insert_single_triangle(
                        m,
                        m.m_vertex_attribute,
                        vertices,
                        faces[face_id],
                        marked_tet_faces,
                        try_acquire_triangle,
                        try_acquire_edge,
                        try_acquire_tetra);
                    if (!success) return false;
                    for (auto& f : marked_tet_faces) tet_face_tags[f].push_back(face_id);
                    return true;
                });
            }); // tg.run
        } // parallel for loop
    });
    arena.execute([&] { tg.wait(); });

    wmtk::logger().info("expired size: {}", expired_queue.size());

    auto check_acquire = [](const auto&) { return true; };

    auto supply_element = [&Q = expired_queue,
                           &face_id_cache =
                               triangle_insertion_local_cache.local().face_id](const auto& func) {
        std::tuple<double, int, size_t> eiq;
        while (Q.try_pop(eiq)) {
            const auto& [_, retry_time, face_id] = eiq;

            face_id_cache = face_id;
            if (func(face_id) == false) {
                continue;
            };
        }
    };

    supply_element([&](auto face_id) {
        std::vector<std::array<size_t, 3>> marked_tet_faces;
        auto success = internal_insert_single_triangle(
            *this,
            m_vertex_attribute,
            vertices,
            faces[face_id],
            marked_tet_faces,
            check_acquire,
            check_acquire,
            check_acquire);
        if (!success) return false;
        for (auto& f : marked_tet_faces) tet_face_tags[f].push_back(face_id);
        return true;
    });

    //// track surface, bbox, rounding
    wmtk::logger().info("finished insertion");

    setup_attributes(vertices, faces, tet_face_tags);

    wmtk::logger().info("setup attributes #t {} #v {}", tet_capacity(), vert_capacity());
} // note: skip preserve open boundaries

void tetwild::TetWild::setup_attributes(
    const std::vector<Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces,
    const tbb::concurrent_map<std::array<size_t, 3>, std::vector<int>>& tet_face_tags)
{
    tbb::task_arena arena(NUM_THREADS);

    arena.execute([&vertices, &faces, &tet_face_tags, this] {
        tbb::parallel_for(
            tet_face_tags.range(),
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