
#include <wmtk/TetMesh.h>

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/TupleUtils.hpp>

#include <algorithm>
#include <cstdio>
#include <iterator>
#include <vector>

// TODO: this is a generic function, move to utils?
void replace(std::array<size_t, 4>& arr, size_t v0, size_t v1)
{
    for (auto j = 0; j < arr.size(); j++) {
        if (arr[j] == v0) {
            arr[j] = v1;
        }
    }
}

// TODO: this is a generic function, move to utils?
constexpr auto find_other_v = [](auto& tet, auto& verts) {
    std::set<size_t> result(tet.begin(), tet.end());
    for (auto vi : verts) result.erase(vi);

    assert(result.size() == 1);
    return *result.begin();
};

namespace wmtk {


void TetMesh::operation_failure_rollback_imp(
    std::map<size_t, VertexConnectivity>& rollback_vert_conn,
    const std::vector<size_t>& affected,
    const std::vector<size_t>& new_tet_id,
    const std::vector<TetrahedronConnectivity>& old_tets)
{
    for (const size_t ti : new_tet_id) {
        m_tet_connectivity[ti].m_is_removed = true;
        m_tet_connectivity[ti].hash--;
    }
    for (int i = 0; i < affected.size(); i++) {
        m_tet_connectivity[affected[i]] = old_tets[i];
    }
    for (auto& [v, conn] : rollback_vert_conn) {
        m_vertex_connectivity[v] = std::move(conn);
    }

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
std::map<size_t, TetMesh::VertexConnectivity> TetMesh::operation_update_connectivity_impl(
    std::vector<size_t>& remove_id,
    const std::vector<std::array<size_t, 4>>& new_tet_conn)
{
    std::vector<size_t> allocate;
    auto rollback_vert_conn = operation_update_connectivity_impl(remove_id, new_tet_conn, allocate);
    remove_id = allocate;
    return rollback_vert_conn;
}

std::map<size_t, TetMesh::VertexConnectivity> wmtk::TetMesh::operation_update_connectivity_impl(
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


bool TetMesh::swap_edge(const Tuple& t, std::vector<Tuple>& new_tet_tuples)
{
    // 3-2 edge to face.
    // only swap internal edges, not on boundary.
    // if (t.is_boundary_edge(*this)) return false;
    if (!swap_edge_before(t)) {
        return false;
    }
    const size_t v1_id = t.vid(*this);
    const size_t v2_id = switch_vertex(t).vid(*this);
    auto& nb1 = m_vertex_connectivity[v1_id];
    auto& nb2 = m_vertex_connectivity[v2_id];
    const auto affected = set_intersection(nb1.m_conn_tets, nb2.m_conn_tets);
    assert(!affected.empty());
    if (affected.size() != 3) {
        logger().trace("selected edges need 3 neighbors to swap.");
        return false;
    }
    std::set<size_t> verts;
    for (const size_t ti : affected)
        for (auto j = 0; j < 4; j++) {
            verts.insert(m_tet_connectivity[ti][j]);
        }
    if (verts.size() != affected.size() + 2) {
        return false; // boundary
    }

    // get vids for return
    const size_t v_A = t.vid(*this);
    const size_t v_B = t.switch_edge(*this).switch_vertex(*this).vid(*this);
    const size_t v_C = t.switch_face(*this).switch_edge(*this).switch_vertex(*this).vid(*this);
    auto v_D_tuple = t.switch_tetrahedron(*this);
    assert(v_D_tuple.has_value());
    const size_t v_D =
        (*v_D_tuple).switch_face(*this).switch_edge(*this).switch_vertex(*this).vid(*this);

    const auto old_tets = record_old_tet_connectivity(m_tet_connectivity, affected);
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

    // get eid, fid, tid for return
    size_t tid_for_return = -1;
    for (size_t tid_v : m_vertex_connectivity[v_B].m_conn_tets) {
        if (m_tet_connectivity[tid_v].find(v_A) != -1 &&
            m_tet_connectivity[tid_v].find(v_C) != -1 &&
            m_tet_connectivity[tid_v].find(v_D) != -1) {
            tid_for_return = tid_v;
            break;
        }
    }
    assert(tid_for_return != -1);

    const auto eid_for_return = m_tet_connectivity[tid_for_return].find_local_edge(v_B, v_C);
    assert(eid_for_return != -1);
    const auto fid_for_return = m_tet_connectivity[tid_for_return].find_local_face(v_B, v_C, v_D);
    assert(fid_for_return != -1);

    const Tuple newt(*this, v_B, eid_for_return, fid_for_return, tid_for_return);

    for (const size_t ti : new_tet_id) {
        new_tet_tuples.emplace_back(tuple_from_tet(ti));
    }
    start_protect_attributes();
    if (!swap_edge_after(newt) || !invariants(new_tet_tuples)) { // rollback post-operation
        assert(affected.size() == old_tets.size());
        operation_failure_rollback_imp(rollback_vert_conn, affected, new_tet_id, old_tets);
        return false;
    }
    release_protect_attributes();


    return true;
}


std::vector<std::array<size_t, 4>> swap_4_4(
    const std::vector<std::array<size_t, 4>>& tets,
    size_t u0,
    size_t u1,
    size_t v0,
    std::array<size_t, 2>& newedge)
{
    auto find = [](auto& arr, auto v) {
        for (auto i = 0; i < arr.size(); i++) {
            if (arr[i] == v) return i;
        }
        return -1;
    };

    std::set<size_t> verts; // all vertices
    for (auto j = 0; j < 4; j++) {
        for (auto k = 0; k < 4; k++) {
            verts.insert(tets[j][k]);
        }
    }
    // remove old edge from vertices
    verts.erase(u0);
    verts.erase(u1);
    assert(verts.size() == 4);

    auto find_other_v_local = [&](const auto& tet, const auto& tri) {
        for (int i = 0; i < tet.size(); i++) {
            if (tet[i] != tri[0] && tet[i] != tri[1] && tet[i] != tri[2]) {
                return i;
            }
        }
        return -1;
    };

    std::vector<size_t> ss;
    for (int j = 0; j < 4; j++) {
        const std::array<size_t, 3> tri{{v0, u0, u1}};
        if (find(tets[j], v0) != -1) {
            int local_i = find_other_v_local(tets[j], tri);
            assert(local_i != -1);
            ss.push_back(tets[j][local_i]);
        }
    }
    assert(ss.size() == 2);
    const size_t s0 = ss.front();
    const size_t s1 = ss.back();
    verts.erase(v0);
    verts.erase(s0);
    verts.erase(s1);
    assert(verts.size() == 1);
    const size_t v1 = (*verts.begin());

    std::vector<std::array<size_t, 4>> new_tet_conn;
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

bool TetMesh::swap_edge_44(const Tuple& t, std::vector<Tuple>& new_tet_tuples)
{
    // 4-4 edge to face.
    // only swap internal edges, not on boundary.
    // if (t.is_boundary_edge(*this)) return false;
    if (!swap_edge_44_before(t)) {
        return false;
    }
    const SmartTuple tt(*this, t);
    const size_t v1_id = tt.vid();
    const size_t v2_id = tt.switch_vertex().vid();
    const auto& nb1 = m_vertex_connectivity[v1_id];
    const auto& nb2 = m_vertex_connectivity[v2_id];
    auto affected = set_intersection(nb1.m_conn_tets, nb2.m_conn_tets);
    assert(!affected.empty());
    if (affected.size() != 4) {
        logger().trace("selected edges need 4 neighbors to swap.");
        return false;
    }
    std::set<size_t> verts;
    for (auto ti : affected) {
        for (auto j = 0; j < 4; j++) {
            verts.insert(m_tet_connectivity[ti][j]);
        }
    }
    if (verts.size() != affected.size() + 2) {
        return false; // boundary
    }

    // get vids for return
    // TODO this is probably faster when implemented with BFS as in the original TetWild
    const size_t v_A = tt.switch_edge().switch_vertex().vid();
    const size_t v_E = tt.switch_face().switch_edge().switch_vertex().vid();
    const size_t v_D = tt.switch_vertex().vid();
    const auto v_C_tuple = tt.switch_tetrahedron();
    assert(v_C_tuple.has_value());
    const size_t v_C = (*v_C_tuple).switch_face().switch_edge().switch_vertex().vid();
    const auto v_F_tuple = tt.switch_face().switch_tetrahedron();
    assert(v_F_tuple.has_value());
    const size_t v_F = (*v_F_tuple).switch_face().switch_edge().switch_vertex().vid();

    const auto old_tets = record_old_tet_connectivity(m_tet_connectivity, affected);
    auto old_tets_conn = std::vector<std::array<size_t, 4>>();
    for (const auto& ti : old_tets) {
        old_tets_conn.push_back(ti.m_indices);
    }

    std::vector<size_t> new_tet_id;
    bool is_succeed = false;

    std::array<size_t, 2> v0s = {{v_E, v_A}};
    for (size_t v0 : v0s) {
        std::array<size_t, 2> edge_vv;
        // get new tet connectivity
        auto new_tets = swap_4_4(old_tets_conn, v1_id, v2_id, v0, edge_vv);

        new_tet_id = affected;
        // update tet and vertex connectivity
        auto rollback_vert_conn = operation_update_connectivity_impl(new_tet_id, new_tets);
        assert(new_tet_id.size() == 4);

        // build return tuple and gather new tet tuples
        Tuple newt;
        if (v0 == v_A) {
            size_t tid_for_return = -1;
            for (size_t tid_v : m_vertex_connectivity[v_A].m_conn_tets) {
                if (m_tet_connectivity[tid_v].find(v_E) != -1 &&
                    m_tet_connectivity[tid_v].find(v_D) != -1 &&
                    m_tet_connectivity[tid_v].find(v_F) != -1) {
                    tid_for_return = tid_v;
                    break;
                }
            }

            assert(tid_for_return != -1);

            int eid_for_return = m_tet_connectivity[tid_for_return].find_local_edge(v_A, v_F);
            assert(eid_for_return != -1);
            int fid_for_return = m_tet_connectivity[tid_for_return].find_local_face(v_A, v_D, v_F);
            assert(fid_for_return != -1);
            newt = Tuple(*this, v_A, eid_for_return, fid_for_return, tid_for_return);
        } else {
            size_t tid_for_return = -1;
            for (size_t tid_v : m_vertex_connectivity[v_A].m_conn_tets) {
                if (m_tet_connectivity[tid_v].find(v_C) != -1 &&
                    m_tet_connectivity[tid_v].find(v_D) != -1 &&
                    m_tet_connectivity[tid_v].find(v_E) != -1) {
                    tid_for_return = tid_v;
                    break;
                }
            }

            assert(tid_for_return != -1);

            auto eid_for_return = m_tet_connectivity[tid_for_return].find_local_edge(v_E, v_C);
            assert(eid_for_return != -1);
            auto fid_for_return = m_tet_connectivity[tid_for_return].find_local_face(v_E, v_C, v_A);
            assert(fid_for_return != -1);
            newt = Tuple(*this, v_E, eid_for_return, fid_for_return, tid_for_return);
        }

        new_tet_tuples.clear();
        for (const size_t ti : new_tet_id) {
            new_tet_tuples.emplace_back(tuple_from_tet(ti));
        }
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
    if (!is_succeed) {
        return false;
    }

    return true;
}


bool TetMesh::swap_face(const Tuple& t, std::vector<Tuple>& new_tet_tuples)
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

    // get vids for return
    auto v_B = t.vid(*this);
    auto v_C = t.switch_vertex(*this).vid(*this);
    auto v_A = t.switch_face(*this).switch_edge(*this).switch_vertex(*this).vid(*this);
    auto v_E_tuple = t.switch_tetrahedron(*this);
    assert(v_E_tuple.has_value());
    auto v_E = (*v_E_tuple).switch_face(*this).switch_edge(*this).switch_vertex(*this).vid(*this);

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
        // auto new_tid = new_tet_id.front();
        // auto new_eid = m_tet_connectivity[new_tid].find_local_edge(oppo_vid[0], oppo_vid[1]);
        // logger().trace("oppo vid {}", oppo_vid);
        // assert(new_eid != -1);
        // auto newt = tuple_from_edge(new_tid, new_eid);

        // get eid, fid, tid for return
        size_t tid_for_return = -1;
        for (size_t tid_v : m_vertex_connectivity[v_A].m_conn_tets) {
            if (m_tet_connectivity[tid_v].find(v_B) != -1 &&
                m_tet_connectivity[tid_v].find(v_C) != -1 &&
                m_tet_connectivity[tid_v].find(v_E) != -1) {
                tid_for_return = tid_v;
                break;
            }
        }
        assert(tid_for_return != -1);

        auto eid_for_return = m_tet_connectivity[tid_for_return].find_local_edge(v_A, v_E);
        assert(eid_for_return != -1);
        auto fid_for_return = m_tet_connectivity[tid_for_return].find_local_face(v_A, v_E, v_B);
        assert(fid_for_return != -1);

        auto newt = Tuple(*this, v_A, eid_for_return, fid_for_return, tid_for_return);


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

/**
 * @brief Perform the 5-6 swap for the edge (u0,u1)
 *
 * Sketch of 5-6 swap:
 * The edge (u0,u1) is orthogonal to the drawing plane and therefore u0 and u1 are at the same
 * position. Edges connecting u0/u1 with v0/v1/etc. were omitted.
 *
 *   v3 --------- v2              v3 --------- v2
 *     |         |                  |\       /|
 *     |    .    |                  | \     / |
 *     |   u01   |       ===>       | |     | |
 *   v4 \       / v1              v4 \ \   / / v1
 *       \     /                      \|   |/
 *        \   /                        \\ //
 *         v0                           v0
 *
 * New triangles: (v0,v1,v2), (v0,v2,v3), (v0,v3,v4)
 * These triangles plus u0/u1 form the new tets, 3 on each side of the drawing plane.
 * (u0, v0,v1,v2)
 * (u0, v0,v2,v3)
 * (u0, v0,v3,v4)
 * (u1, v0,v2,v1)
 * (u1, v0,v3,v2)
 * (u1, v0,v4,v3)
 */
std::vector<std::array<size_t, 4>> swap_5_6(
    const std::vector<std::array<size_t, 4>>& tets,
    size_t u0,
    size_t u1,
    size_t v0,
    std::array<size_t, 3>& newtri)
{
    // use BFS to find oriented link vertices of (u0,u1)
    std::vector<std::array<int, 3>> u12_link_e; // link edges
    u12_link_e.reserve(tets.size());
    for (int i = 0; i < tets.size(); i++) {
        std::array<int, 3> e;
        int cnt = 0;
        for (int j = 0; j < 4; j++)
            if (tets[i][j] != u0 && tets[i][j] != u1) {
                e[cnt++] = tets[i][j];
            }
        e[cnt] = i;
        u12_link_e.push_back(e);
    }

    std::vector<size_t> n12_vs; // (u0,u1) link vertices (sorted!)
    n12_vs.reserve(5);
    std::vector<size_t> n12_ts; // (u0,u1) tets (sorted!)
    n12_ts.reserve(5);
    std::array<bool, 5> is_visited;
    std::fill(is_visited.begin(), is_visited.end(), false);
    n12_vs.push_back(u12_link_e[0][0]);
    n12_vs.push_back(u12_link_e[0][1]);
    n12_ts.push_back(u12_link_e[0][2]);
    is_visited[0] = true;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 5; j++) {
            if (is_visited[j]) {
                continue;
            }
            if (u12_link_e[j][0] == n12_vs.back()) {
                is_visited[j] = true;
                n12_vs.push_back(u12_link_e[j][1]);
            } else if (u12_link_e[j][1] == n12_vs.back()) {
                is_visited[j] = true;
                n12_vs.push_back(u12_link_e[j][0]);
            }
            if (is_visited[j]) {
                n12_ts.push_back(u12_link_e[j][2]);
                break;
            }
        }
    }
    n12_ts.push_back(
        u12_link_e[std::find(is_visited.begin(), is_visited.end(), false) - is_visited.begin()][2]);
    assert(n12_vs.size() == 5);
    assert(n12_ts.size() == 5);

    auto replace = [](const std::array<size_t, 4>& arr, size_t v_old, size_t v_new) {
        std::array<size_t, 4> out = arr;
        for (auto i = 0; i < arr.size(); i++) {
            if (out[i] == v_old) {
                out[i] = v_new;
                break;
            }
        }
        return out;
    };

    {
        // index of v0
        int i = std::distance(n12_vs.begin(), std::find(n12_vs.begin(), n12_vs.end(), v0));

        // auto it = std::find(n12_vs.begin(), n12_vs.end(), v0);
        // std::rotate(n12_vs.begin(), it, n12_vs.end());
        auto tet_left = tets[n12_ts[(i - 1 + 5) % 5]];
        auto tet_right = tets[n12_ts[i]];
        auto tet_opp = tets[n12_ts[(i + 2) % 5]];


        std::vector<std::array<size_t, 4>> new_tet_conn;
        new_tet_conn.reserve(6);
        auto tet_l1 = replace(tet_left, u0, n12_vs[(i - 2 + 5) % 5]);
        auto tet_l2 = replace(tet_left, u1, n12_vs[(i - 2 + 5) % 5]);
        auto tet_r1 = replace(tet_right, u0, n12_vs[(i + 2 + 5) % 5]);
        auto tet_r2 = replace(tet_right, u1, n12_vs[(i + 2 + 5) % 5]);
        auto tet_o1 = replace(tet_opp, u0, n12_vs[i]);
        auto tet_o2 = replace(tet_opp, u1, n12_vs[i]);
        new_tet_conn.push_back(tet_l1);
        new_tet_conn.push_back(tet_l2);
        new_tet_conn.push_back(tet_r1);
        new_tet_conn.push_back(tet_r2);
        new_tet_conn.push_back(tet_o1);
        new_tet_conn.push_back(tet_o2);

        newtri = {{n12_vs[i], n12_vs[(i + 2) % 5], n12_vs[(i - 2 + 5) % 5]}};
        return new_tet_conn;
    }
}

bool TetMesh::swap_edge_56(const Tuple& t, std::vector<Tuple>& new_tet_tuples)
{
    // only swap internal edges, not on boundary.
    // if (t.is_boundary_edge(*this)) return false;
    if (!swap_edge_56_before(t)) {
        return false;
    }
    const SmartTuple tt(*this, t);
    const size_t v1_id = tt.vid();
    const size_t v2_id = tt.switch_vertex().vid();
    const auto& nb1 = m_vertex_connectivity[v1_id];
    const auto& nb2 = m_vertex_connectivity[v2_id];
    auto affected = set_intersection(nb1.m_conn_tets, nb2.m_conn_tets);
    assert(!affected.empty());
    if (affected.size() != 5) {
        logger().trace("selected edges need 4 neighbors to swap.");
        return false;
    }
    std::set<size_t> verts;
    for (auto ti : affected) {
        for (auto j = 0; j < 4; j++) {
            verts.insert(m_tet_connectivity[ti][j]);
        }
    }
    if (verts.size() != affected.size() + 2) {
        return false; // boundary
    }
    verts.erase(v1_id);
    verts.erase(v2_id);

    const auto old_tets = record_old_tet_connectivity(m_tet_connectivity, affected);
    auto old_tets_conn = std::vector<std::array<size_t, 4>>();
    for (const auto& ti : old_tets) {
        old_tets_conn.push_back(ti.m_indices);
    }

    std::vector<size_t> new_tet_id;
    bool is_succeed = false;

    for (const size_t v0 : verts) {
        std::array<size_t, 3> face_vv;
        // get new tet connectivity
        auto new_tets = swap_5_6(old_tets_conn, v1_id, v2_id, v0, face_vv);
        assert(v0 == face_vv[0]);

        new_tet_id = affected;
        // update tet and vertex connectivity
        auto rollback_vert_conn = operation_update_connectivity_impl(new_tet_id, new_tets);
        assert(new_tet_id.size() == 6);

        // build return tuple and gather new tet tuples
        // Tuple newt = tuple_from_vids(face_vv[0], face_vv[1], face_vv[2], v1_id);
        const auto& vf0 = m_vertex_connectivity[face_vv[0]];
        const auto& vf1 = m_vertex_connectivity[face_vv[1]];
        const auto& vf2 = m_vertex_connectivity[face_vv[2]];
        const auto& vf3 = m_vertex_connectivity[v1_id];
        const std::vector<size_t> tets01 = set_intersection(vf0.m_conn_tets, vf1.m_conn_tets);
        const std::vector<size_t> tets012 = set_intersection(tets01, vf2.m_conn_tets);
        const std::vector<size_t> tets0123 = set_intersection(tets012, vf3.m_conn_tets);
        if (tets0123.size() != 1) {
            // The swap can create a tet with the same indices as an already existing tet. That case
            // is prohibited here.
            operation_failure_rollback_imp(rollback_vert_conn, affected, new_tet_id, old_tets);
            continue;
        }
        const size_t tid = tets0123[0];
        const size_t eid = m_tet_connectivity[tid].find_local_edge(face_vv[0], face_vv[1]);
        const size_t fid =
            m_tet_connectivity[tid].find_local_face(face_vv[0], face_vv[1], face_vv[2]);

        Tuple newt(*this, v0, eid, fid, tid);

        new_tet_tuples.clear();
        for (const size_t ti : new_tet_id) {
            new_tet_tuples.emplace_back(tuple_from_tet(ti));
        }
        start_protect_attributes();
        if (!swap_edge_56_after(newt) || !invariants(new_tet_tuples)) { // rollback post-operation
            assert(affected.size() == old_tets.size());
            operation_failure_rollback_imp(rollback_vert_conn, affected, new_tet_id, old_tets);
            continue;
        }
        release_protect_attributes();
        is_succeed = true;
        break;
    }
    if (!is_succeed) {
        return false;
    }

    return true;
}

} // namespace wmtk