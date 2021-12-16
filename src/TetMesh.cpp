//
// Created by Yixin Hu on 11/3/21.
//

#include <wmtk/TetMesh.h>

#include <wmtk/TupleUtils.hpp>

// DP: I do not understand the logic here
int wmtk::TetMesh::find_next_empty_slot_t() // todo: always append in the end
{
    for (int i = m_t_empty_slot; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) {
            m_t_empty_slot = i + 1;
            return i;
        }
    }
    m_tet_connectivity.emplace_back();
    return m_tet_connectivity.size() - 1;
}

int wmtk::TetMesh::find_next_empty_slot_v()
{
    for (int i = m_v_empty_slot; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) {
            m_v_empty_slot = i + 1;
            return i;
        }
    }
    m_vertex_connectivity.emplace_back();
    return m_vertex_connectivity.size() - 1;
}

void wmtk::TetMesh::init(size_t n_vertices, const std::vector<std::array<size_t, 4>>& tets)
{
    m_vertex_connectivity.resize(n_vertices);
    m_tet_connectivity.resize(tets.size());
    for (int i = 0; i < tets.size(); i++) {
        m_tet_connectivity[i].m_indices = tets[i];
        for (int j = 0; j < 4; j++) {
            assert(tets[i][j] < m_vertex_connectivity.size());
            m_vertex_connectivity[tets[i][j]].m_conn_tets.push_back(i);
        }
    }
}

bool wmtk::TetMesh::swap_edge(const Tuple& t)
{
    // 3-2 edge to face.
    // only swap internal edges, not on boundary.
    if (t.is_boundary_edge(*this)) return false;
    if (!swap_edge_before(t)) return false;
    auto v1_id = t.vid();
    auto v2_id = switch_vertex(t).vid();
    auto& nb1 = m_vertex_connectivity[v1_id];
    auto& nb2 = m_vertex_connectivity[v2_id];
    auto affected = set_intersection(nb1.m_conn_tets, nb2.m_conn_tets);
    assert(!affected.empty());
    if (affected.size() != 3) {
        logger().trace("selected edges need 3 neighbors to swap.");
        return false;
    }

    using tet_conn_t = decltype(TetrahedronConnectivity::m_indices);
    auto old_tets = [&tet_attrs = m_tet_connectivity, &tets = affected]() {
        auto tet_conn = std::vector<TetrahedronConnectivity>();
        for (auto i : tets) tet_conn.push_back(tet_attrs[i]);
        return tet_conn;
    }();

    auto new_tets = [&tet_attrs = m_tet_connectivity, v1_id, v2_id, &affected]() {
        auto t0_id = affected[0];
        auto t1_id = affected[1];
        auto t2_id = affected[2];
        auto n0_id = -1, n1_id = -1, n2_id = -1;
        for (int j = 0; j < 4; j++) {
            auto v0j = tet_attrs[t0_id][j];
            if (v0j != v1_id && v0j != v2_id) {
                if (tet_attrs[t1_id].find(v0j) != -1) n1_id = v0j;
                if (tet_attrs[t2_id].find(v0j) != -1) n2_id = v0j;
            }
            if (tet_attrs[t0_id].find(tet_attrs[t1_id].m_indices[j]) == -1)
                n0_id = tet_attrs[t1_id].m_indices[j];
        }
        assert(n0_id != n1_id && n1_id != n2_id);
        // T0 = (n1,n2,v1,v2) -> (n1,n2,v1,n0)
        // T1 = (n0, n1, v1,v2) ->  (n0, n1, n2,v2)
        // T2 = (n0,n2, v1,v2) -> (-1,-1,-1,-1)
        auto new_tets = std::vector<tet_conn_t>(2);
        auto new_tids = std::vector<size_t>({t0_id, t1_id});
        auto replace = [](auto& arr, auto v0, auto v1) {
            for (auto j = 0; j < arr.size(); j++)
                if (arr[j] == v0) arr[j] = v1;
        };
        new_tets[0] = tet_attrs[new_tids[0]].m_indices;
        new_tets[1] = tet_attrs[new_tids[1]].m_indices;

        replace(new_tets[0], v2_id, n0_id);
        replace(new_tets[1], v1_id, n2_id);
        return new_tets;
    }();

    auto update_connectivity = [](auto& tet_conn,
                                  auto& vert_conn,
                                  auto& remove_id,
                                  std::vector<tet_conn_t>& new_tet_conn) {
        auto new_tid = std::vector<size_t>();
        auto affected_vid = std::set<size_t>();
        std::sort(remove_id.begin(), remove_id.end());
        for (auto i : remove_id) {
            tet_conn[i].m_is_removed = true;
            auto& conn = tet_conn[i].m_indices;
            for (auto j = 0; j < 4; j++) {
                affected_vid.insert(conn[j]);
            }
        }
        std::map<size_t, VertexConnectivity> rollback_vert_conn;
        for (auto v : affected_vid) rollback_vert_conn.emplace(v, vert_conn[v]); // here is a copy

        for (auto& conn : new_tet_conn) {
            auto tid = tet_conn.size();
            new_tid.push_back(tid);
            auto new_tet = TetrahedronConnectivity();
            new_tet.m_indices = conn;
            tet_conn.emplace_back(new_tet);
            for (auto j = 0; j < 4; j++) {
                auto vid = conn[j];
                assert(affected_vid.find(vid) != affected_vid.end() && "not introducing new verts");
                assert(vert_conn.size() > vid && "Sufficient number of verts");
                vert_conn[vid].m_conn_tets.push_back(tid);
            }
        }

        for (auto v : affected_vid) {
            auto new_tets = decltype(VertexConnectivity::m_conn_tets)();
            for (auto t : vert_conn[v].m_conn_tets) {
                if (!tet_conn[t].m_is_removed) {
                    new_tets.push_back(t);
                }
            }
            std::sort(new_tets.begin(), new_tets.end());
            vert_conn[v].m_conn_tets = std::move(new_tets);
        }
        return rollback_vert_conn;
    };

    auto rollback_vert_conn =
        update_connectivity(m_tet_connectivity, m_vertex_connectivity, affected, new_tets);

    resize_attributes(
        m_vertex_connectivity.size(),
        m_tet_connectivity.size() * 6,
        m_tet_connectivity.size() * 4,
        m_tet_connectivity.size());

    if (!swap_edge_after(t)) { // rollback post-operation
        assert(affected.size() == old_tets.size());
        for (auto i = 0; i < affected.size(); i++) m_tet_connectivity[affected[i]] = old_tets[i];
        for (auto& [v, conn] : rollback_vert_conn) m_vertex_connectivity[v] = std::move(conn);
        return false;
    }

    return true;
}


std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_edges() const
{
    std::vector<TetMesh::Tuple> edges;
    for (int i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 6; j++) {
            edges.push_back(tuple_from_edge(i, j));
        }
    }

    unique_edge_tuples(*this, edges);

    return edges;
}

bool wmtk::TetMesh::check_mesh_connectivity_validity() const
{
    std::vector<std::vector<size_t>> conn_tets(m_vertex_connectivity.size());
    for (size_t i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 4; j++) conn_tets[m_tet_connectivity[i][j]].push_back(i);
    }

    // check conn_tets duplication, order, amount ...
    for (size_t i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        assert(
            m_vertex_connectivity[i].m_conn_tets == conn_tets[i] &&
            "m_vertex_connectivity[i].m_conn_tets!=conn_tets[i]");
    }

    // check is_removed
    for (size_t i = 0; i < m_tet_connectivity.size(); i++) {
        for (int j = 0; j < 4; j++)
            assert(
                !m_vertex_connectivity[m_tet_connectivity[i][j]].m_is_removed &&
                "m_vertex_connectivity[m_tet_connectivity[i][j]].m_is_removed");
    }
    for (size_t i = 0; i < m_vertex_connectivity.size(); i++) {
        for (int tid : m_vertex_connectivity[i].m_conn_tets)
            assert(!m_tet_connectivity[tid].m_is_removed && "m_tet_connectivity[tid].m_is_removed");
    }

    // check tuple
    for (size_t i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        Tuple loc = tuple_from_vertex(i);
        check_tuple_validity(loc);
        //
        Tuple locv = switch_vertex(loc);
        Tuple loce = switch_edge(loc);
        Tuple locf = switch_face(loc);
        auto loct = switch_tetrahedron(loc);
        check_tuple_validity(locv);
        check_tuple_validity(loce);
        check_tuple_validity(locf);
        if (loct.has_value()) {
            check_tuple_validity(loct.value());
        }
    }
    for (size_t i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        Tuple loc = tuple_from_tet(i);
        check_tuple_validity(loc);
        //
        Tuple locv = switch_vertex(loc);
        Tuple loce = switch_edge(loc);
        Tuple locf = switch_face(loc);
        auto loct = switch_tetrahedron(loc);
        check_tuple_validity(locv);
        check_tuple_validity(loce);
        check_tuple_validity(locf);
        if (loct.has_value()) {
            check_tuple_validity(loct.value());
        }
    }

    return true;
}
