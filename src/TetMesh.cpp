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

bool wmtk::TetMesh::swap_edge(const Tuple& t) {
    // 3-2 edge to face.
    auto v1_id= t.vid();
    auto v2_id = switch_vertex(t).vid();
    auto& nb1 = m_vertex_connectivity[v1_id];
    auto& nb2 = m_vertex_connectivity[v2_id];
    auto affected = set_intersection(nb1.m_conn_tets, nb2.m_conn_tets);
    assert(!affected.empty());
    if (affected.size() != 3) {
        return false;
    }

    // TODO: if (!bnd_faces.empty()) return false; // NOT handling boundary edges for now.

    auto new_tets = [&tet_attrs=m_tet_connectivity, v1_id, v2_id, &affected]() {
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


    return true;
}



std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_edges() const
{
    std::vector<TetMesh::Tuple> edges;
    for (int i = 0; i < m_tet_connectivity.size(); i++) {
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

    //check conn_tets duplication, order, amount ...
    for (size_t i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        assert(
            m_vertex_connectivity[i].m_conn_tets == conn_tets[i] &&
            "m_vertex_connectivity[i].m_conn_tets!=conn_tets[i]");
    }

    //check is_removed
    for (size_t i = 0; i < m_tet_connectivity.size(); i++) {
        for(int j=0;j<4;j++)
            assert(!m_vertex_connectivity[m_tet_connectivity[i][j]].m_is_removed
                   &&"m_vertex_connectivity[m_tet_connectivity[i][j]].m_is_removed");
    }
    for (size_t i = 0; i < m_vertex_connectivity.size(); i++) {
        for (int tid : m_vertex_connectivity[i].m_conn_tets)
            assert(!m_tet_connectivity[tid].m_is_removed && "m_tet_connectivity[tid].m_is_removed");
    }

    //check tuple
    for (size_t i = 0; i < m_vertex_connectivity.size(); i++) {
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
        if(loct.has_value()) {
            check_tuple_validity(loct.value());
        }
    }
    for (size_t i = 0; i < m_tet_connectivity.size(); i++) {
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
        if(loct.has_value()) {
            check_tuple_validity(loct.value());
        }
    }

    return true;
}
