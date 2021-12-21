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

bool wmtk::TetMesh::collapse_edge(const Tuple& t, std::vector<Tuple>& new_edges)
{
    // todo

    return true;
}


std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_vertices() const
{
    std::vector<TetMesh::Tuple> edges;
    for (auto i = 0; i < m_vertex_connectivity.size(); i++) {
        auto& vc = m_vertex_connectivity[i];
        if (vc.m_is_removed) continue;
        assert(!vc.m_conn_tets.empty());
        auto tid = vc[0];
        auto local_vid = m_tet_connectivity[tid].find(i);
        
        // note: the following conversion of local_vid-eid is **heavily** dependent on the specifics of
        // `m_local_edges`
        auto local_eid = local_vid;
        if (local_vid >= 2) local_eid = 5; 
        edges.emplace_back(tuple_from_edge(tid, local_eid));
        if (local_vid == 3) edges.back() = switch_vertex(edges.back());
        edges.back().update_version_number(*this);
        assert(edges.back().vid() == i);
        assert(edges.back().is_version_number_valid(*this));
    }
    return edges;
}

bool wmtk::TetMesh::smooth_vertex(const Tuple& loc0)
{
    if (!smooth_before(loc0)) return false;
    if (!smooth_after(loc0)) return false;

    return true;
}