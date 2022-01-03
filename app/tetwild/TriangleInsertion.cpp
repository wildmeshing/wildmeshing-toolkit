//
// Created by Yixin Hu on 1/3/22.
//

#include "Logger.hpp"
#include "TetWild.h"
#include "wmtk/TetMesh.h"

template<typename T>
bool segment_triangle_intersection(std::array<T, 2>& seg, std::array<T, 3>& tri, T& p){//todo
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

    for (size_t face_id = 0; face_id < faces.size(); face_id++) {
        is_visited.assign(m_tet_attribute.size(), false); // reset

        std::array<Vector3, 3> tri = {
            {to_rational(vertices[faces[face_id][0]]),
             to_rational(vertices[faces[face_id][1]]),
             to_rational(vertices[faces[face_id][2]])}};

        std::vector<size_t> intersected_tids;
        std::map<std::array<size_t, 2>, std::pair<bool, Vector3>> map_edge2point;


        std::queue<Tuple> tet_queue;
        while (!tet_queue.empty()) {
            auto tet = tet_queue.front();
            tet_queue.pop();

            // todo: get 6 edges of tet t_id
            std::array<Tuple, 6> edges;

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
                if (!is_intersected) {
                    map_edge2point[e] = std::make_pair(false, p);
                    continue;
                } else {
                    map_edge2point[e] = std::make_pair(true, p);
                    need_subdivision = true;
                }

                // add new tets
                auto incident_tets = get_incident_tets_for_edge(loc);
                for (auto& t : incident_tets) tet_queue.push(t);
            }

            if (need_subdivision) intersected_tids.push_back(tet.tid(*this));
        }
        wmtk::vector_unique(intersected_tids);
        for (auto it = map_edge2point.begin(), ite = map_edge2point.end(); it != ite;) {//erase edge without intersections
            if (!(it->second).first)
                it = map_edge2point.erase(it);
            else
                ++it;
        }

        // todo: subdivide tets --> should be move to conn??
        for (size_t tet_id : intersected_tids) {
            // todo: get 6 edges of tet t_id
            std::array<Tuple, 6> edges;

            std::bitset<6> config;
            for (int i = 0; i < edges.size(); i++) {
                auto& loc = edges[i];
                size_t v1_id = loc.vid(*this);
                auto tmp = switch_vertex(loc);
                size_t v2_id = tmp.vid(*this);
                std::array<size_t, 2> e = {v1_id, v2_id};
                if (e[0] > e[1]) std::swap(e[0], e[1]);

                if (!map_edge2point.count(e)) continue;
                auto info = map_edge2point[e];
                if (!info.first) //if false ==> already pushed back
                    continue;

                // todo: add new vertex (conn & attr
//                m_vertex_attribute.push_back()

                config.set(i);

                //todo: subdivide tet
            }
        }
    }
}