//
// Created by Yixin Hu on 1/3/22.
//

#include "Logger.hpp"
#include "TetWild.h"
#include "wmtk/TetMesh.h"
#include "wmtk/auto_table.hpp"

template<typename T3>
bool segment_triangle_intersection(const std::array<T3, 2>& seg, const std::array<T3, 3>& tri, T3& p){//todo: open segment
    return false;
}

template<typename T2, typename T3>
void squeeze_points_to_2d(const std::vector<T3>& points3, const std::vector<T2>& points2){//todo
}

template<typename T2>
bool is_point_inside_triangle(const T2& p, const std::array<T2, 3>& tri){//todo
    return false;
}

tetwild::Vector3 to_rational(const tetwild::Vector3d& p0){//todo
    tetwild::Vector3 p;
    return p;
}

void tetwild::TetWild::triangle_insertion(std::vector<Vector3d>& vertices,
                                          std::vector<std::array<size_t, 3>>& faces)
{
    std::vector<bool> is_visited;

    triangle_insertion_cache.surface_f_ids.resize(m_tet_attribute.size(), {{-1, -1, -1, -1}});

    for (size_t face_id = 0; face_id < faces.size(); face_id++) {
        is_visited.assign(m_tet_attribute.size(), false); // reset

        std::array<Vector3, 3> tri = {
            {to_rational(vertices[faces[face_id][0]]),
             to_rational(vertices[faces[face_id][1]]),
             to_rational(vertices[faces[face_id][2]])}};

        std::vector<size_t> intersected_tids;
        std::map<std::array<size_t, 2>, std::pair<int, Vector3>> map_edge2point;


        std::queue<Tuple> tet_queue;
        while (!tet_queue.empty()) {
            auto tet = tet_queue.front();
            tet_queue.pop();

            std::array<Tuple, 6> edges = tet_edges(tet);

            // check if the edge intersects with the triangle
            bool need_subdivision = false;
            for (auto& loc : edges) {
                size_t v1_id = loc.vid(*this);
                auto tmp = switch_vertex(loc);
                size_t v2_id = tmp.vid(*this);
                std::array<size_t, 2> e = {v1_id, v2_id};
                if (e[0] > e[1]) std::swap(e[0], e[1]);

                if (map_edge2point.count(e)) {
                    if (map_edge2point[e].first) need_subdivision = true;
                    continue;
                }

                std::array<Vector3, 2> seg = {
                    {m_vertex_attribute[v1_id].m_pos, m_vertex_attribute[v2_id].m_pos}};
                Vector3 p(0, 0, 0);
                bool is_intersected = segment_triangle_intersection(seg, tri, p);
                map_edge2point[e] = std::make_pair(is_intersected, p);
                if (!is_intersected) {
                    continue;
                } else {
                    need_subdivision = true;
                }

                // add new tets
                auto incident_tets = get_incident_tets_for_edge(loc);
                for (auto& t : incident_tets) tet_queue.push(t);
            }

            if (need_subdivision) intersected_tids.push_back(tet.tid(*this));
        }
        wmtk::vector_unique(intersected_tids);
        for (auto it = map_edge2point.begin(), ite = map_edge2point.end();
             it != ite;) { // erase edge without intersections
            if (!(it->second).first)
                it = map_edge2point.erase(it);
            else
                ++it;
        }

        ///push back new vertices
        std::map<std::array<size_t, 2>, size_t> map_edge2vid;
        int v_cnt = 0;
        for (auto& info : map_edge2point) {
            VertexAttributes v;
            v.m_pos = (info.second).second;
            m_vertex_attribute.push_back(v);
            (info.second).first = m_vertex_attribute.size() - 1;
            v_cnt++;

            map_edge2vid[info.first] = (info.second).first;
        }

        ///push back new tets conn
        triangle_insertion_cache.face_id = face_id;
        subdivide_tets(intersected_tids, map_edge2vid); // in TetMesh class

        ///resize attri lists
        m_tet_attribute.resize(tets_size()); // todo: ???
    }

    /// update m_is_on_surface for vertices, remove leaked surface marks
    m_edge_attribute.resize(m_tet_attribute.size() * 6);
    m_face_attribute.resize(m_tet_attribute.size() * 4);
    for (int i = 0; i < triangle_insertion_cache.surface_f_ids.size(); i++) {
        Tuple tet = tuple_from_tet(i);
        auto tet_vertices = oriented_tet_vertices(tet);
        for (int j = 0; j < 4; j++) {
            size_t face_id = triangle_insertion_cache.surface_f_ids[i][j];
            if (face_id < 0) continue;

            Vector3 c = m_vertex_attribute[tet_vertices[(j + 1) % 4].vid(*this)].m_pos +
                        m_vertex_attribute[tet_vertices[(j + 2) % 4].vid(*this)].m_pos +
                        m_vertex_attribute[tet_vertices[(j + 3) % 4].vid(*this)].m_pos;
            c = c / 3;

            std::vector<Vector3> ps3 = {
                c,
                to_rational(vertices[faces[face_id][0]]),
                to_rational(vertices[faces[face_id][1]]),
                to_rational(vertices[faces[face_id][2]])};
            std::vector<Vector2> ps2;
            squeeze_points_to_2d(ps3, ps2);

            if (is_point_inside_triangle(ps2[0], {{ps2[1], ps2[2], ps2[3]}})) {
                Tuple face = tuple_from_face(i, j);
                int global_fid = face.fid(*this);
                m_face_attribute[global_fid].m_surface_tags = face_id;
            }
        }
    }

    /// todo: track bbox

    // note: skip preserve open boundaries
}

void tetwild::TetWild::insertion_update_surface_tag(size_t t_id, size_t new_t_id,
                                  int config_id, int diag_config_id, int index)
{
    const auto& config = floatTetWild::CutTable::get_tet_conf(config_id, diag_config_id);
    const auto& new_is_surface_fs =
        floatTetWild::CutTable::get_surface_conf(config_id, diag_config_id);
    const auto& old_local_f_ids =
        floatTetWild::CutTable::get_face_id_conf(config_id, diag_config_id);

    // track surface ==> t_id, new_t_id, is_surface_fs for new_t_id, local_f_ids of t_id mapped to new_t_id, face_id (cached)
    triangle_insertion_cache.surface_f_ids.emplace_back();
    auto& new_surface_f_ids = triangle_insertion_cache.surface_f_ids.back();
    //
    auto old_surface_f_ids = triangle_insertion_cache.surface_f_ids[t_id];
    //
    for (int j = 0; j < 4; j++) {
        if (old_surface_f_ids[old_local_f_ids[index][j]] >= 0)
            new_surface_f_ids[j] = old_surface_f_ids[old_local_f_ids[index][j]];

        if (new_is_surface_fs[index][j]) new_surface_f_ids[j] = triangle_insertion_cache.face_id;
        // note: new face_id has higher priority than old ones
    }
}