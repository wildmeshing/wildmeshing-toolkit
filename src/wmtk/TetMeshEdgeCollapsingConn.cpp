#include <wmtk/TetMesh.h>

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/TupleUtils.hpp>

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <type_traits>

bool wmtk::TetMesh::collapse_edge(const Tuple& loc0, std::vector<Tuple>& new_edges)
{
    if (!collapse_edge_before(loc0)) return false;

    auto link_condition = [&VC = this->m_vertex_connectivity,
                           &TC = this->m_tet_connectivity,
                           this](auto v0, auto v1) -> bool {
        auto intersects = [](const auto& vec, const auto& val) {
            for (auto& v : vec)
                for (auto& a : val)
                    if (v == a) return true;
            return false;
        };

        auto link = [&intersects,
                     &conn = TC](const auto& verts, const auto& tets, const auto& faces) {
            auto conn_verts = std::set<size_t>();
            auto conn_edges = std::set<std::array<size_t, 2>>();
            auto conn_faces = std::set<std::array<size_t, 3>>();
            logger().trace("In Link {}", verts);
            auto collect = [&](auto& tet) {
                for (auto j = 0; j < 4; j++) {
                    auto vj = tet[j];
                    if (intersects(std::array<size_t, 1>{{vj}}, verts)) continue;
                    conn_verts.insert(vj);
                }
                for (auto e : m_local_edges) {
                    auto e0 = tet[e[0]], e1 = tet[e[1]];
                    auto edge = std::array<size_t, 2>{{std::min(e0, e1), std::max(e0, e1)}};
                    if (intersects(edge, verts)) continue;
                    conn_edges.emplace(edge);
                }
                for (auto f : m_local_faces) {
                    auto face = std::array<size_t, 3>{{tet[f[0]], tet[f[1]], tet[f[2]]}};
                    if (intersects(face, verts)) continue;
                    std::sort(face.begin(), face.end());
                    conn_faces.emplace(face);
                }
            };
            for (auto& t : tets) {
                collect(conn[t]);
            }
            const auto dummy = std::numeric_limits<size_t>::max();
            logger().trace("in face {}", faces);
            for (auto& f : faces) {
                auto tet = std::array<size_t, 4>{{f[0], f[1], f[2], dummy}};
                collect(tet);
            }
            logger().trace("out points {}", conn_verts);
            return std::tuple<
                std::set<size_t>,
                std::set<std::array<size_t, 2>>,
                std::set<std::array<size_t, 3>>>{conn_verts, conn_edges, conn_faces};
        };
        auto& closure0 = VC[v0].m_conn_tets;
        auto& closure1 = VC[v1].m_conn_tets;
        auto bnd0 = vertex_adjacent_boundary_faces(this->tuple_from_vertex(v0));
        auto bnd1 = vertex_adjacent_boundary_faces(this->tuple_from_vertex(v1));
        auto lk0 = link(std::array<size_t, 1>{{v0}}, closure0, bnd0);
        auto lk1 = link(std::array<size_t, 1>{{v1}}, closure1, bnd1);

        auto bnd01 = set_intersection(bnd0, bnd1);

        // link of edge
        auto common_tets = set_intersection(closure0, closure1);
        auto lk_01 = link(std::array<size_t, 2>{{v0, v1}}, common_tets, bnd01);
        if (!std::get<2>(lk_01).empty()) return false;

        auto lk_i = std::tuple<
            std::vector<size_t>,
            std::vector<std::array<size_t, 2>>,
            std::vector<std::array<size_t, 3>>>();
        auto check_inter = [](const auto& l0, const auto& l1, const auto& l01, auto& l_i) {
            // link(v0) intersect link(v1) == link(edge 01)
            wmtk::logger().trace(">> Checking l0 {} l1 {}, l01 {}", l0, l1, l01);
            std::set_intersection(
                l0.begin(),
                l0.end(),
                l1.begin(),
                l1.end(),
                std::back_inserter(l_i));
            return (
                l_i.size() == l01.size() &&
                std::equal(l_i.begin(), l_i.end(), l01.begin(), l01.end()));
        };
        if (!check_inter(std::get<0>(lk0), std::get<0>(lk1), std::get<0>(lk_01), std::get<0>(lk_i)))
            return false;
        if (!check_inter(std::get<1>(lk0), std::get<1>(lk1), std::get<1>(lk_01), std::get<1>(lk_i)))
            return false;
        if (!check_inter(std::get<2>(lk0), std::get<2>(lk1), std::get<2>(lk_01), std::get<2>(lk_i)))
            return false;

        return true;
    };

    /// backup of everything
    auto v1_id = loc0.vid(*this);
    auto loc1 = switch_vertex(loc0);
    auto v2_id = loc1.vid(*this);
    logger().trace("{} {}", v1_id, v2_id);
    if (m_collapse_check_link_condition && !link_condition(v1_id, v2_id)) {
        wmtk::logger().trace("violate link condition");
        return false;
    }

    // should be a copy, for the purpose of rollback
    auto n1_t_ids =
        m_vertex_connectivity[v1_id].m_conn_tets; // note: conn_tets for v1 without removed tets
    const auto& n2_t_ids = m_vertex_connectivity[v2_id].m_conn_tets;

    std::set<std::array<size_t, 4>> verify_conns; // simplified manifold topology check.
    for (auto _t : n2_t_ids) {
        auto tet = m_tet_connectivity[_t].m_indices;
        std::sort(tet.begin(), tet.end());
        verify_conns.emplace(tet);
    }

    auto new_tet_conn = std::vector<std::array<size_t, 4>>();
    new_tet_conn.reserve(n1_t_ids.size());
    std::vector<TetrahedronConnectivity> old_tets;
    std::vector<size_t> preserved_tids;
    for (auto t_id : n1_t_ids) {
        old_tets.push_back(m_tet_connectivity[t_id]);
        int l1 = -1, l2 = -1;
        for (int j = 0; j < 4; j++) {
            if (m_tet_connectivity[t_id][j] == v1_id)
                l1 = j;
            else if (m_tet_connectivity[t_id][j] == v2_id) {
                l2 = j;
                break;
            }
        }
        if (l2 != -1) continue;
        assert(l1 != -1);
        new_tet_conn.push_back(m_tet_connectivity[t_id].m_indices);
        new_tet_conn.back()[l1] = v2_id;
        preserved_tids.push_back(t_id);
    }
    {
        for (std::array<size_t, 4> tet : new_tet_conn) {
            std::sort(tet.begin(), tet.end());
            auto [it, suc] = verify_conns.emplace(tet);
            if (!suc) { // duplicate
                return false;
            }
        }
    }

    auto rollback_vert_conn =
        operation_update_connectivity_impl(n1_t_ids, new_tet_conn, preserved_tids);

    auto& new_tet_id = preserved_tids;
    assert(rollback_vert_conn.find(v1_id) != rollback_vert_conn.end());
    //
    m_vertex_connectivity[v1_id].m_is_removed = true;
    m_vertex_connectivity[v1_id].m_conn_tets.clear();

    Tuple new_loc = tuple_from_vertex(v2_id);


    auto check_topology = [&]() {
        for (size_t t_id : new_tet_id)
            for (auto k = 0; k < 4; k++) {
                std::array<size_t, 3> vids;
                for (auto j = 0; j < 3; j++)
                    vids[j] = m_tet_connectivity[t_id].m_indices[(k + 1 + j) % 4];
                auto [_, fid] = tuple_from_face(vids);
                if (fid == -1) {
                    return false;
                }
            }
        return true;
    };

    start_protect_attributes();
    if (!check_topology() || !collapse_edge_after(new_loc) ||
        !invariants(get_one_ring_tets_for_vertex(new_loc))) {
        m_vertex_connectivity[v1_id].m_is_removed = false;
        operation_failure_rollback_imp(rollback_vert_conn, n1_t_ids, new_tet_id, old_tets);

        return false;
    }

    release_protect_attributes();

    for (size_t t_id : new_tet_id) {
        for (int j = 0; j < 6; j++) {
            new_edges.push_back(tuple_from_edge(t_id, j));
        }
    }
    unique_edge_tuples(*this, new_edges);

    return true;
}