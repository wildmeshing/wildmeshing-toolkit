//
// Created by Yixin Hu on 1/3/22.
//

#include <wmtk/utils/Delaunay.hpp>
#include "Logger.hpp"
#include "TetWild.h"
#include "wmtk/TetMesh.h"
#include "wmtk/auto_table.hpp"
#include "wmtk/utils/GeoUtils.h"

#include <igl/remove_duplicate_vertices.h>

using std::cout;
using std::endl;

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

    apps::logger().info("remove duplicates: ");
    apps::logger().info("#v: {} -> {}", vertices.size(), V_in.rows());
    apps::logger().info("#f: {} -> {}", faces.size(), F_in.rows());

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
    double delta = m_params.diag_l / 10.0;
    Vector3d box_min(m_params.min[0] - delta, m_params.min[1] - delta, m_params.min[2] - delta);
    Vector3d box_max(m_params.max[0] + delta, m_params.max[1] + delta, m_params.max[2] + delta);
    // todo: add voxel points
    points.push_back({{box_min[0], box_min[1], box_min[2]}});
    points.push_back({{box_max[0], box_max[1], box_max[2]}});

    cout<<m_params.min.transpose()<<endl;
    cout<<m_params.max.transpose()<<endl;
    cout<<box_min.transpose()<<endl;
    cout<<box_max.transpose()<<endl;
//    apps::logger().info("min: {}", m_params.min);
//    apps::logger().info("max: {}", m_params.max.transpose());
//    apps::logger().info("box_min: {}", box_min.transpose());
//    apps::logger().info("box_max: {}", box_max.transpose());

    ///delaunay
    auto tets = wmtk::delaunay3D_conn(points);
    cout<<"tets.size() "<<tets.size()<<endl;

    // conn
    init(points.size(), tets);
    // attr
    resize_attributes(points.size(), tets.size() * 6, tets.size() * 4, tets.size());
    for (int i = 0; i < m_vertex_attribute.size(); i++) {
        m_vertex_attribute[i].m_pos = Vector3(points[i][0], points[i][1], points[i][2]);
        m_vertex_attribute[i].m_posf = Vector3d(points[i][0], points[i][1], points[i][2]);
    }
    // todo: track bbox
}

void tetwild::TetWild::match_insertion_faces(const InputSurface& input_surface, std::vector<bool>& is_matched)
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
    for (size_t i = 0; i < m_tet_attribute.size(); i++) {
        Tuple t = tuple_from_tet(i);
        auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            std::array<size_t, 3> f = {
                {vs[(j + 1) % 3].vid(*this),
                 vs[(j + 2) % 3].vid(*this),
                 vs[(j + 3) % 3].vid(*this)}};
            std::sort(f.begin(), f.end());
            if (map_surface.count(f)) {
                int fid = map_surface[f];
                triangle_insertion_cache.surface_f_ids[i][j] = fid;
                is_matched[fid] = true;
            }
        }
    }
}

void tetwild::TetWild::triangle_insertion(const InputSurface& input_surface)
{
    //fortest
    auto pausee = []() {
        std::cout << "pausing..." << std::endl;
        char c;
        std::cin >> c;
        if (c == '0') exit(0);
    };
    auto print = [](const Vector3& p){
        cout<<p[0]<<" "<<p[1]<<" "<<p[2]<<endl;
    };
    //fortest

    construct_background_mesh(input_surface);

    const auto& vertices = input_surface.vertices;
    const auto& faces = input_surface.faces;

    triangle_insertion_cache.surface_f_ids.resize(m_tet_attribute.size(), {{-1, -1, -1, -1}});
    //match faces preserved in delaunay
    std::vector<bool> is_matched;
    match_insertion_faces(input_surface, is_matched);
    cout<<"is_matched "<<std::count(is_matched.begin(), is_matched.end(), true)<<endl;

    std::vector<bool> is_visited;
    for (size_t face_id = 0; face_id < faces.size(); face_id++) {
        if(is_matched[face_id])
            continue;

        is_visited.assign(m_tet_attribute.size(), false); // reset

        std::array<Vector3, 3> tri = {
            {to_rational(vertices[faces[face_id][0]]),
             to_rational(vertices[faces[face_id][1]]),
             to_rational(vertices[faces[face_id][2]])}};
        std::array<Vector2, 3> tri2;
        int squeeze_to_2d_dir = wmtk::squeeze_triangle_to_2d(tri, tri2);

        std::vector<size_t> intersected_tids;
        std::map<std::array<size_t, 2>, std::pair<int, Vector3>> map_edge2point;

        cout<<"face_id "<<face_id<<endl;
//        print(tri[0]);
//        print(tri[1]);
//        print(tri[2]);

        std::queue<Tuple> tet_queue;
        //
        for(int j=0;j<3;j++) {
            Tuple loc = tuple_from_vertex(faces[face_id][j]);
            auto conn_tets = get_one_ring_tets_for_vertex(loc);
            for (const auto& t : conn_tets) {
                tet_queue.push(t);
                is_visited[t.tid(*this)] = true;
            }
        }
        //
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
                std::array<size_t, 2> e = {{v1_id, v2_id}};
                if (e[0] > e[1]) std::swap(e[0], e[1]);

                if (map_edge2point.count(e)) {
                    if (map_edge2point[e].first) need_subdivision = true;
                    continue;
                }

                std::array<Vector3, 2> seg = {
                    {m_vertex_attribute[v1_id].m_pos, m_vertex_attribute[v2_id].m_pos}};
                Vector3 p(0, 0, 0);
                bool is_coplanar = wmtk::segment_triangle_coplanar_3d(seg, tri);
                bool is_intersected = false;
//                if(is_coplanar) {
                if(true) {
                    std::array<Vector2, 2> seg2;
                    seg2[0] = wmtk::squeeze_point_to_2d(seg[0], squeeze_to_2d_dir);
                    seg2[1] = wmtk::squeeze_point_to_2d(seg[1], squeeze_to_2d_dir);
                    for (int j = 0; j < 3; j++) {
                        apps::Rational t1;
                        std::array<Vector2, 2> tri_seg2 = {{tri2[j], tri2[(j + 1) % 3]}};
                        is_intersected = wmtk::open_segment_open_segment_intersection_2d(seg2, tri_seg2, t1);
                        if (is_intersected) {
                            p = (1 - t1) * seg[0] + t1 * seg[1];
                            break;
                        }
                    }
                } else {
                    is_intersected = wmtk::open_segment_triangle_intersection_3d(seg, tri, p);
                }
//                print(seg[0]);
//                print(seg[1]);
                cout<<tet.tid(*this)<<" is_intersected "<<is_intersected<<endl;

                map_edge2point[e] = std::make_pair(is_intersected, p);
                if (!is_intersected) {
                    continue;
                } else {
                    need_subdivision = true;
                }

                // add new tets
                auto incident_tets = get_incident_tets_for_edge(loc);
                for (auto& t : incident_tets) {
                    int tid = t.tid(*this);
                    if(is_visited[tid])
                        continue;
                    tet_queue.push(t);
                    is_visited[tid] = true;
                }
            }

            if (need_subdivision) intersected_tids.push_back(tet.tid(*this));
        }
        wmtk::vector_unique(intersected_tids);
        cout<<"intersected_tids.size "<<intersected_tids.size()<<endl;

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
        cout<<"map_edge2vid.size "<<map_edge2vid.size()<<endl;

        ///push back new tets conn
        triangle_insertion_cache.face_id = face_id;
        subdivide_tets(intersected_tids, map_edge2vid); // in TetMesh class

        ///resize attri lists
        m_tet_attribute.resize(tets_size()); // todo: ???

        check_mesh_connectivity_validity();
//        pausee();
    }
    pausee();

    /// update m_is_on_surface for vertices, remove leaked surface marks
    m_edge_attribute.resize(m_tet_attribute.size() * 6);
    m_face_attribute.resize(m_tet_attribute.size() * 4);
    for (int i = 0; i < triangle_insertion_cache.surface_f_ids.size(); i++) {
        Tuple tet = tuple_from_tet(i);
        auto tet_vertices = oriented_tet_vertices(tet);
        for (int j = 0; j < 4; j++) {
            int face_id = triangle_insertion_cache.surface_f_ids[i][j];
            if (face_id < 0) continue;

            Vector3 c = m_vertex_attribute[tet_vertices[(j + 1) % 4].vid(*this)].m_pos +
                        m_vertex_attribute[tet_vertices[(j + 2) % 4].vid(*this)].m_pos +
                        m_vertex_attribute[tet_vertices[(j + 3) % 4].vid(*this)].m_pos;
            c = c / 3;

            std::array<Vector3, 3> tri = {{
                to_rational(vertices[faces[face_id][0]]),
                to_rational(vertices[faces[face_id][1]]),
                to_rational(vertices[faces[face_id][2]])}};
            std::array<Vector2, 3> tri2;
            int squeeze_to_2d_dir = wmtk::squeeze_triangle_to_2d(tri, tri2);
            auto c2 = wmtk::squeeze_point_to_2d(c, squeeze_to_2d_dir);

            if (wmtk::is_point_inside_triangle(c2, tri2)) {
                Tuple face = tuple_from_face(i, j);
                int global_fid = face.fid(*this);
                m_face_attribute[global_fid].m_surface_tags = face_id;
            }
        }
    }

    /// todo: track bbox

    // note: skip preserve open boundaries
}

void tetwild::TetWild::insertion_update_surface_tag(
    size_t t_id,
    size_t new_t_id,
    int config_id,
    int diag_config_id,
    int index)
{
    const auto& config = wmtk::CutTable::get_tet_conf(config_id, diag_config_id);
    const auto& new_is_surface_fs = wmtk::CutTable::get_surface_conf(config_id, diag_config_id);
    const auto& old_local_f_ids = wmtk::CutTable::get_face_id_conf(config_id, diag_config_id);

    // track surface ==> t_id, new_t_id, is_surface_fs for new_t_id, local_f_ids of t_id mapped to
    // new_t_id, face_id (cached)
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