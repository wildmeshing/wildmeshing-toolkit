
#include <wmtk/TetMesh.h>

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/TupleUtils.hpp>

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <vector>

// TODO: this is a generic function, move to utils?
auto replace(std::array<size_t, 4>& arr, size_t v0, size_t v1)
{
    for (auto j = 0; j < arr.size(); j++)
        if (arr[j] == v0) arr[j] = v1;
}

// TODO: this is a generic function, move to utils?
constexpr auto find_other_v = [](auto& tet, auto& verts) {
    std::set<size_t> result(tet.begin(), tet.end());
    for (auto vi : verts) result.erase(vi);

    assert(result.size() == 1);
    return *result.begin();
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

    rollback_protected_attributes();
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
std::map<size_t, wmtk::TetMesh::VertexConnectivity>
wmtk::TetMesh::operation_update_connectivity_impl(
    std::vector<size_t>& remove_id,
    const std::vector<std::array<size_t, 4>>& new_tet_conn)
{
    std::vector<size_t> allocate;
    auto rollback_vert_conn = operation_update_connectivity_impl(remove_id, new_tet_conn, allocate);
    remove_id = allocate;
    return rollback_vert_conn;
}

std::map<size_t, wmtk::TetMesh::VertexConnectivity>
wmtk::TetMesh::operation_update_connectivity_impl(
    const std::vector<size_t>& remove_id,
    const std::vector<std::array<size_t, 4>>& new_tet_conn,
    std::vector<size_t>& allocate_id)
{
    // TODO: special case with fixed id.
    assert(allocate_id.empty() || allocate_id.size() == new_tet_conn.size());
    assert(std::is_sorted(remove_id.begin(), remove_id.end()));


    auto& tet_conn = this->m_tet_connectivity;
    auto& vert_conn = this->m_vertex_connectivity;
    auto new_tid = std::vector<size_t>();
    auto affected_vid = std::set<size_t>();
    for (auto i : remove_id) {
        tet_conn[i].m_is_removed = true;
        auto& conn = tet_conn[i].m_indices;
        for (auto j = 0; j < 4; j++) {
            affected_vid.insert(conn[j]);
        }
    }
    std::map<size_t, wmtk::TetMesh::VertexConnectivity> rollback_vert_conn;
    for (auto v : affected_vid) {
        rollback_vert_conn.emplace(v, vert_conn[v]); // here is a copy
    }
    for (auto i : remove_id) {
        auto& conn = tet_conn[i].m_indices;
        for (auto j = 0; j < 4; j++) {
            auto flag = wmtk::set_erase(vert_conn[conn[j]].m_conn_tets, i);
        }
    }

    if (allocate_id.empty()) {
        allocate_id = remove_id;
        if (new_tet_conn.size() <= allocate_id.size()) { // tet number decrease
            allocate_id.resize(new_tet_conn.size());
        } else {
            auto hole_size = allocate_id.size();

            auto add_size = new_tet_conn.size() - allocate_id.size();
            allocate_id.resize(new_tet_conn.size(), -1);

            auto new_indices = std::vector<size_t>(add_size);
            // auto old_tet_size = tet_conn.size();
            for (auto i = 0; i < add_size; i++) {
                allocate_id[i + hole_size] = this->get_next_empty_slot_t(); // old_tet_size + i;
            }
        }
    }
    assert(allocate_id.size() == new_tet_conn.size());

    for (auto i = 0; i < new_tet_conn.size(); i++) {
        auto id = allocate_id[i];
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
}


bool wmtk::TetMesh::swap_edge(const Tuple& t, std::vector<Tuple>& new_tet_tuples)
{
    // 3-2 edge to face.
    // only swap internal edges, not on boundary.
    // if (t.is_boundary_edge(*this)) return false;
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
    std::set<size_t> verts;
    for (auto ti : affected)
        for (auto j = 0; j < 4; j++) verts.insert(m_tet_connectivity[ti][j]);
    if (verts.size() != affected.size() + 2) return false; // boundary

    auto old_tets = record_old_tet_connectivity(m_tet_connectivity, affected);
    auto new_tets = std::vector<std::array<size_t, 4>>(2);
    {
        auto& tet_conn = m_tet_connectivity;
        auto t0_id = affected[0];
        auto t1_id = affected[1];
        auto t2_id = affected[2];
        auto n0_id = -1, n1_id = -1, n2_id = -1;
        for (int j = 0; j < 4; j++) {
            auto v0j = tet_conn[t0_id][j];
            if (v0j != v1_id && v0j != v2_id) {
                if (tet_conn[t1_id].find(v0j) != -1) n1_id = v0j;
                if (tet_conn[t2_id].find(v0j) != -1) n2_id = v0j;
            }
            if (tet_conn[t0_id].find(tet_conn[t1_id].m_indices[j]) == -1)
                n0_id = tet_conn[t1_id].m_indices[j];
        }
        assert(n0_id != n1_id && n1_id != n2_id);
        // T0 = (n1,n2,v1,*v2*) -> (n1,n2,v1,*n0*)
        // T1 = (n0, n1, *v1*,v2) ->  (n0, n1, *n2*,v2)
        // T2 = (n0,n2, v1,v2) -> (-1,-1,-1,-1)
        {
            auto inter = set_intersection(
                m_vertex_connectivity[n0_id].m_conn_tets,
                m_vertex_connectivity[n1_id].m_conn_tets);
            inter = set_intersection(m_vertex_connectivity[n2_id].m_conn_tets, inter);
            if (!inter.empty()) return false;
        }

        new_tets[0] = tet_conn[t0_id].m_indices;
        new_tets[1] = tet_conn[t1_id].m_indices;

        replace(new_tets[0], v2_id, n0_id);
        replace(new_tets[1], v1_id, n2_id);
    }

    auto new_tet_id = affected;
    auto rollback_vert_conn = operation_update_connectivity_impl(new_tet_id, new_tets);
    assert(new_tet_id.size() == 2);

    auto u0id = m_tet_connectivity[new_tet_id.front()].find(v1_id);
    assert(u0id != -1);
    auto newt = tuple_from_face(new_tet_id.front(), m_map_vertex2oppo_face[u0id]);

    for (auto ti : new_tet_id) new_tet_tuples.emplace_back(tuple_from_tet(ti));
    start_protect_attributes();
    if (!swap_edge_after(newt) || !invariants(new_tet_tuples)) { // rollback post-operation
        assert(affected.size() == old_tets.size());
        operation_failure_rollback_imp(rollback_vert_conn, affected, new_tet_id, old_tets);
        return false;
    }
    release_protect_attributes();


    return true;
}


auto swap_4_4(
    const std::vector<std::array<size_t, 4>>& tets,
    size_t u0,
    size_t u1,
    int type,
    std::array<size_t, 2>& newedge)
{
    auto n0 = -1, n1 = -1, n2 = -1, n3 = -1;

    auto find = [](auto& arr, auto v) {
        for (auto i = 0; i < arr.size(); i++) {
            if (arr[i] == v) return i;
        }
        return -1;
    };
    std::set<size_t> verts;
    for (auto j = 0; j < 4; j++) {
        for (auto k = 0; k < 4; k++) {
            verts.insert(tets[j][k]);
        }
    }
    verts.erase(u0);
    verts.erase(u1);
    assert(verts.size() == 4);

    auto v0 = (*verts.begin());
    auto find_other_v_local = [&](const auto& tet, const auto& tri) {
        for (auto i = 0; i < tet.size(); i++) {
            if (tet[i] != tri[0] && tet[i] != tri[1] && tet[i] != tri[2]) return i;
        }
        return -1;
    };
    auto ss = std::vector<size_t>();
    for (auto j = 0; j < 4; j++) {
        auto tri = std::array<size_t, 3>{{v0, u0, u1}};
        if (find(tets[j], v0) != -1) {
            auto local_i = find_other_v_local(tets[j], tri);
            assert(local_i != -1);
            ss.push_back(tets[j][local_i]);
        }
    }
    assert(ss.size() == 2);
    auto s0 = ss.front(), s1 = ss.back();
    verts.erase(v0);
    verts.erase(s0);
    verts.erase(s1);
    assert(verts.size() == 1);
    auto v1 = (*verts.begin());
    if (type == 1) {
        std::swap(v0, s0);
        std::swap(v1, s1);
    }

    auto new_tet_conn = std::vector<std::array<size_t, 4>>();
    for (auto j = 0; j < 4; j++) {
        if (find(tets[j], v1) != -1) {
            new_tet_conn.push_back(tets[j]);
            replace(new_tet_conn.back(), u0, v0);
        } else {
            new_tet_conn.push_back(tets[j]);
            replace(new_tet_conn.back(), u1, v1);
        }
    }
    assert(new_tet_conn.size() == 4);
    newedge = {{v0, v1}};
    return new_tet_conn;
}

bool wmtk::TetMesh::swap_edge_44(const Tuple& t, std::vector<Tuple>& new_tet_tuples)
{
    // 4-4 edge to face.
    // only swap internal edges, not on boundary.
    // if (t.is_boundary_edge(*this)) return false;
    if (!swap_edge_44_before(t)) return false;
    auto v1_id = t.vid(*this);
    auto v2_id = switch_vertex(t).vid(*this);
    auto& nb1 = m_vertex_connectivity[v1_id];
    auto& nb2 = m_vertex_connectivity[v2_id];
    auto affected = set_intersection(nb1.m_conn_tets, nb2.m_conn_tets);
    assert(!affected.empty());
    if (affected.size() != 4) {
        logger().trace("selected edges need 4 neighbors to swap.");
        return false;
    }
    std::set<size_t> verts;
    for (auto ti : affected)
        for (auto j = 0; j < 4; j++) verts.insert(m_tet_connectivity[ti][j]);
    if (verts.size() != affected.size() + 2) return false; // boundary

    auto old_tets = record_old_tet_connectivity(m_tet_connectivity, affected);
    auto old_tets_conn = std::vector<std::array<size_t, 4>>();
    for (auto& ti : old_tets) old_tets_conn.push_back(ti.m_indices);

    std::vector<size_t> new_tet_id;
    bool is_succeed = false;
    for (int type = 0; type < 2; type++) {
        auto edge_vv = std::array<size_t, 2>();
        auto new_tets = swap_4_4(old_tets_conn, v1_id, v2_id, type, edge_vv);

        new_tet_id = affected;
        auto rollback_vert_conn = operation_update_connectivity_impl(new_tet_id, new_tets);
        assert(new_tet_id.size() == 4);

        auto new_tuple_from_edge = [&]() -> Tuple {
            auto tid = new_tet_id.front();
            auto j0 = m_tet_connectivity[tid].find(edge_vv[0]);
            auto j1 = m_tet_connectivity[tid].find(edge_vv[1]);
            assert(j0 != -1 && j1 != -1);
            if (j0 > j1) std::swap(j0, j1);
            auto edge_v = std::array<int, 2>{{j0, j1}};
            auto it = std::find(m_local_edges.begin(), m_local_edges.end(), edge_v);
            auto eid = std::distance(m_local_edges.begin(), it);
            return tuple_from_edge(tid, eid);
        };
        auto newt = new_tuple_from_edge();

        for (auto ti : new_tet_id) new_tet_tuples.emplace_back(tuple_from_tet(ti));
        start_protect_attributes();
        if (!swap_edge_44_after(newt) || !invariants(new_tet_tuples)) { // rollback post-operation
            assert(affected.size() == old_tets.size());
            operation_failure_rollback_imp(rollback_vert_conn, affected, new_tet_id, old_tets);
            continue;
        }
        release_protect_attributes();
        is_succeed = true;
        break;
    }
    if (!is_succeed) return false;

    return true;
}


bool wmtk::TetMesh::swap_face(const Tuple& t, std::vector<Tuple>& new_tet_tuples)
{
    {
        if (t.is_boundary_face(*this)) return false;
        if (!swap_face_before(t)) return false;
    }

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
    std::vector<std::array<size_t, 4>> new_tets;
    {
        auto t0 = affected.front(), t1 = affected.back();

        std::vector<size_t> tri{v0, v1, v2};
        auto u0 = find_other_v(m_tet_connectivity[t0].m_indices, tri);
        auto u1 = find_other_v(m_tet_connectivity[t1].m_indices, tri);
        oppo_vid = {{u0, u1}};
        //
        new_tets.resize(3, m_tet_connectivity[t0].m_indices);
        for (auto i = 0; i < 3; i++) {
            replace(new_tets[i], tri[i], u1);
        }
    }


    { // check if edge already exist: topological un-swappable
        for (auto ti : m_vertex_connectivity[oppo_vid[0]].m_conn_tets) {
            if (m_tet_connectivity[ti].find(oppo_vid[1]) != -1) {
                return false; // edge already exists
            }
        }
    }

    {
        auto new_tet_id = affected;
        auto rollback_vert_conn = operation_update_connectivity_impl(new_tet_id, new_tets);

        assert(affected.size() == old_tets.size());
        auto new_tid = new_tet_id.front();
        auto new_eid = m_tet_connectivity[new_tid].find_local_edge(oppo_vid[0], oppo_vid[1]);
        logger().trace("oppo vid {}", oppo_vid);
        assert(new_eid != -1);
        auto newt = tuple_from_edge(new_tid, new_eid);

        for (auto ti : new_tet_id) new_tet_tuples.emplace_back(tuple_from_tet(ti));

        start_protect_attributes();
        if (!swap_face_after(newt) || !invariants(new_tet_tuples)) { // rollback post-operation

            logger().trace("rolling back");
            operation_failure_rollback_imp(rollback_vert_conn, affected, new_tet_id, old_tets);
            return false;
        }
        release_protect_attributes();
    }

    logger().trace("swapped");
    return true;
}
