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
            assert(tets[i][j] >= m_vertex_connectivity.size());
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
