
#include <wmtk/TetMesh.h>

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <vector>
#include <wmtk/utils/TupleUtils.hpp>

auto replace = [](auto& arr, auto v0, auto v1) {
    for (auto j = 0; j < arr.size(); j++)
        if (arr[j] == v0) arr[j] = v1;
};

std::vector<wmtk::TetMesh::TetrahedronConnectivity> record_old_tet_connectivity(
    const wmtk::TetMesh::vector<wmtk::TetMesh::TetrahedronConnectivity>& tet_attrs,
    const std::vector<size_t>& tets)
{
    auto tet_conn = std::vector<wmtk::TetMesh::TetrahedronConnectivity>();
    for (auto i : tets) tet_conn.push_back(tet_attrs[i]);
    return tet_conn;
};

void wmtk::TetMesh::operation_failure_rollback_imp(
    std::map<size_t, wmtk::TetMesh::VertexConnectivity>& rollback_vert_conn,
    const std::vector<size_t>& affected,
    const std::vector<size_t>& new_tet_id,
    const std::vector<wmtk::TetMesh::TetrahedronConnectivity>& old_tets)
{
    for (auto ti : new_tet_id) {
        m_tet_connectivity[ti].m_is_removed = true;
        m_tet_connectivity[ti].hash--;
    }
    for (auto i = 0; i < affected.size(); i++) m_tet_connectivity[affected[i]] = old_tets[i];
    for (auto& [v, conn] : rollback_vert_conn) m_vertex_connectivity[v] = std::move(conn);
}

/**
 * @brief
 *
 * @param tet_conn
 * @param vert_conn
 * @param remove_id: this will be modified as new_tet_id, useful for rollback etc.
 * @param new_tet_conn
 * @return std::map<size_t, wmtk::TetMesh::VertexConnectivity>
 */
std::map<size_t, wmtk::TetMesh::VertexConnectivity> wmtk::TetMesh::operation_update_connectivity_impl(
    std::vector<size_t>& remove_id,
    std::vector<std::array<size_t, 4>>& new_tet_conn)
{
    auto& tet_conn = this->m_tet_connectivity;
    auto& vert_conn = this->m_vertex_connectivity;
    auto new_tid = std::vector<size_t>();
    auto affected_vid = std::set<size_t>();
    assert(std::is_sorted(remove_id.begin(), remove_id.end()));
    for (auto i : remove_id) {
        tet_conn[i].m_is_removed = true;
        auto& conn = tet_conn[i].m_indices;
        for (auto j = 0; j < 4; j++) {
            affected_vid.insert(conn[j]);
        }
    }
    std::map<size_t, wmtk::TetMesh::VertexConnectivity> rollback_vert_conn;
    for (auto v : affected_vid) rollback_vert_conn.emplace(v, vert_conn[v]); // here is a copy
    for (auto i : remove_id) {
        auto& conn = tet_conn[i].m_indices;
        for (auto j = 0; j < 4; j++) {
            auto flag = wmtk::set_erase(vert_conn[conn[j]].m_conn_tets, i);
        }
    }

    if (new_tet_conn.size() <= remove_id.size()) { // tet number decrease
        remove_id.resize(new_tet_conn.size());
    } else {
        auto hole_size = remove_id.size();

        auto add_size = new_tet_conn.size() - remove_id.size();
        remove_id.resize(new_tet_conn.size(), -1);

        auto new_indices = std::vector<size_t>(add_size);
        // auto old_tet_size = tet_conn.size();
        for (auto i = 0; i < add_size; i++) {
            remove_id[i + hole_size] = this->get_next_empty_slot_t(); // old_tet_size + i;
        }
    }
    assert(remove_id.size() == new_tet_conn.size());

    for (auto i = 0; i < new_tet_conn.size(); i++) {
        auto id = remove_id[i]; // reuse.
        tet_conn[id].m_indices = new_tet_conn[i];
        tet_conn[id].m_is_removed = false;
        tet_conn[id].hash++;
        for (auto j = 0; j < 4; j++) {
            auto vid = new_tet_conn[i][j];
            assert(vert_conn.size() > vid && "Sufficient number of verts");
            wmtk::set_insert(vert_conn[vid].m_conn_tets, id);
        }
    }

    return rollback_vert_conn;
};


bool wmtk::TetMesh::swap_edge(const Tuple& t, Tuple& newt)
{
    // 3-2 edge to face.
    // only swap internal edges, not on boundary.
    if (t.is_boundary_edge(*this)) return false;
    if (!swap_edge_before(t)) return false;
    auto v1_id = t.vid(*this);
    auto v2_id = switch_vertex(t).vid(*this);
    auto& nb1 = m_vertex_connectivity[v1_id];
    auto& nb2 = m_vertex_connectivity[v2_id];
    auto affected = set_intersection(nb1.m_conn_tets, nb2.m_conn_tets);
    assert(!affected.empty());
    if (affected.size() != 3) {
        logger().trace("selected edges need 3 neighbors to swap.");
        return false;
    }

    auto old_tets = record_old_tet_connectivity(m_tet_connectivity, affected);

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
        auto new_tets = std::vector<std::array<size_t, 4>>(2);
        auto new_tids = std::vector<size_t>({t0_id, t1_id});

        new_tets[0] = tet_attrs[new_tids[0]].m_indices;
        new_tets[1] = tet_attrs[new_tids[1]].m_indices;

        replace(new_tets[0], v2_id, n0_id);
        replace(new_tets[1], v1_id, n2_id);
        return new_tets;
    }();
    auto new_tet_id = affected;
    auto rollback_vert_conn =
        operation_update_connectivity_impl(new_tet_id, new_tets);
    assert(new_tet_id.size() == 2);

    auto u0id = m_tet_connectivity[new_tet_id.front()].find(v1_id);
    assert(u0id != -1);
    newt = tuple_from_face(new_tet_id.front(), m_map_vertex2oppo_face[u0id]);

    if (!swap_edge_after(newt)) { // rollback post-operation
        assert(affected.size() == old_tets.size());
        operation_failure_rollback_imp(
            rollback_vert_conn,
            affected,
            new_tet_id,
            old_tets);
        return false;
    }

    return true;
}


bool wmtk::TetMesh::swap_face(const Tuple& t, Tuple& newt)
{
    if (t.is_boundary_face(*this)) return false;
    if (!swap_face_before(t)) return false;

    auto v0 = t.vid(*this);
    auto oppo = switch_vertex(t);
    auto v1 = oppo.vid(*this);
    auto v2 = oppo.switch_edge(*this).switch_vertex(*this).vid(*this);
    assert(v0 != v1 && v1 != v2 && v2 != v0);
    logger().trace("Swap the face with vertices [{}] [{}] [{}]", v0, v1, v2);
    auto inter01 = set_intersection(
        m_vertex_connectivity[v0].m_conn_tets,
        m_vertex_connectivity[v1].m_conn_tets);
    auto affected = set_intersection(inter01, m_vertex_connectivity[v2].m_conn_tets);

    if (affected.size() == 1) return false; // not handling boundary facets
    logger().trace("affected {}", affected);
    assert(affected.size() == 2);
    auto oppo_vid = std::array<size_t, 2>();
    auto old_tets = record_old_tet_connectivity(m_tet_connectivity, affected);
    auto new_tets = [&affected, v0, v1, v2, &m_tet_connectivity = m_tet_connectivity, &oppo_vid]() {
        auto t0 = affected.front(), t1 = affected.back();

        auto find_other_v = [](auto& tet, auto& tri_set) {
            std::set<size_t> tet_set(tet.begin(), tet.end());
            std::vector<size_t> result(1);
            std::set_difference(
                tet_set.begin(),
                tet_set.end(),
                tri_set.begin(),
                tri_set.end(),
                result.begin());
            assert(result.size() == 1);
            return result.front();
        };
        std::set<size_t> tri{v0, v1, v2};
        auto u0 = find_other_v(m_tet_connectivity[t0].m_indices, tri);
        auto u1 = find_other_v(m_tet_connectivity[t1].m_indices, tri);
        oppo_vid = {{u0, u1}};
        //
        auto new_tets = std::vector<std::array<size_t, 4>>{
            m_tet_connectivity[t0].m_indices,
            m_tet_connectivity[t0].m_indices,
            m_tet_connectivity[t0].m_indices};
        replace(new_tets[0], v0, u1);
        replace(new_tets[1], v1, u1);
        replace(new_tets[2], v2, u1);
        return new_tets;
    }();

    // check if edge already exist: topological un-swappable
    for (auto ti : m_vertex_connectivity[oppo_vid[0]].m_conn_tets) {
        if (m_tet_connectivity[ti].find(oppo_vid[1]) != -1) {
            return false; // edge already exists
        }
    }

    auto new_tet_id = affected;
    auto rollback_vert_conn =
        operation_update_connectivity_impl(new_tet_id, new_tets);

    assert(affected.size() == old_tets.size());
    auto new_tid = new_tet_id.front();
    auto new_eid = m_tet_connectivity[new_tid].find_local_edge(oppo_vid[0], oppo_vid[1]);
    logger().trace("oppo vid {}", oppo_vid);
    assert(new_eid != -1);
    newt = tuple_from_edge(new_tid, new_eid);
    if (!swap_face_after(newt)) { // rollback post-operation
        logger().trace("rolling back");
        operation_failure_rollback_imp(
            rollback_vert_conn,
            affected,
            new_tet_id,
            old_tets);
        return false;
    }
    logger().trace("swapped");

    return true;
}
