
#include "EdgeMeshOperationExecutor.hpp"

namespace wmtk {
// constructor
EdgeMesh::EdgeMeshOperationExecutor::EdgeMeshOperationExecutor(
    EdgeMesh& m,
    const Tuple& operating_tuple,
    Accessor<int64_t>& hash_acc)
    : flag_accessors{{m.get_flag_accessor(PrimitiveType::Vertex), m.get_flag_accessor(PrimitiveType::Edge)}}
    , ee_accessor(m.create_accessor<int64_t>(m.m_ee_handle))
    , ev_accessor(m.create_accessor<int64_t>(m.m_ev_handle))
    , ve_accessor(m.create_accessor<int64_t>(m.m_ve_handle))
    , hash_accessor(hash_acc)
    , m_mesh(m)
{
    m_operating_tuple = operating_tuple;
    Tuple operating_tuple_switch_vertex = m_mesh.switch_vertex(operating_tuple);
    // store ids of incident vertices
    m_operating_edge_id = m_mesh.id_edge(m_operating_tuple);
    m_spine_vids[0] = m_mesh.id_vertex(m_operating_tuple);
    m_spine_vids[1] = m_mesh.id_vertex(operating_tuple_switch_vertex);

    // update hash on neighborhood
    cell_ids_to_update_hash.emplace_back(m_mesh.id_edge(m_operating_tuple));
    if (!m_mesh.is_boundary_vertex(m_operating_tuple)) {
        m_neighbor_eids[0] = m_mesh.id_edge(m_mesh.switch_edge(m_operating_tuple));
        cell_ids_to_update_hash.emplace_back(m_neighbor_eids[0]);
    }
    if (!m_mesh.is_boundary_vertex(operating_tuple_switch_vertex)) {
        m_neighbor_eids[1] = m_mesh.id_edge(m_mesh.switch_edge(operating_tuple_switch_vertex));
        cell_ids_to_update_hash.emplace_back(m_neighbor_eids[1]);
    }

    if (m_neighbor_eids[0] == m_neighbor_eids[1] && m_neighbor_eids[0] == m_operating_edge_id) {
        m_is_self_loop = true;
    }
}

void EdgeMesh::EdgeMeshOperationExecutor::delete_simplices()
{
    for (size_t d = 0; d < simplex_ids_to_delete.size(); ++d) {
        for (const int64_t id : simplex_ids_to_delete[d]) {
            flag_accessors[d].index_access().scalar_attribute(id) = 0;
        }
    }
}

void EdgeMesh::EdgeMeshOperationExecutor::update_cell_hash()
{
    m_mesh.update_cell_hashes(cell_ids_to_update_hash, hash_accessor);
}

const std::array<std::vector<int64_t>, 2>
EdgeMesh::EdgeMeshOperationExecutor::get_split_simplices_to_delete(
    const Tuple& tuple,
    const EdgeMesh& m)
{
    std::array<std::vector<int64_t>, 2> ids;
    ids[1].emplace_back(m.id_edge(tuple));
    return ids;
}

const std::array<std::vector<int64_t>, 2>
EdgeMesh::EdgeMeshOperationExecutor::get_collapse_simplices_to_delete(
    const Tuple& tuple,
    const EdgeMesh& m)
{
    std::array<std::vector<int64_t>, 2> ids;
    ids[0].emplace_back(m.id_vertex(tuple));
    ids[1].emplace_back(m.id_edge(tuple));
    return ids;
}


void EdgeMesh::EdgeMeshOperationExecutor::split_edge()
{
    m_output_tuple = split_edge_single_mesh();
    // TODO: Implement for multi_mesh in the future
}

Tuple EdgeMesh::EdgeMeshOperationExecutor::split_edge_single_mesh()
{
    simplex_ids_to_delete = get_split_simplices_to_delete(m_operating_tuple, m_mesh);

    // create new vertex
    const std::vector<int64_t> new_vids = this->request_simplex_indices(PrimitiveType::Vertex, 1);
    assert(new_vids.size() == 1);
    const int64_t v_new = new_vids[0];
    m_split_v = v_new;
    // create new edges
    // new_eids[i] is connect to m_neighbor_eids[i] and m_spine_vids[i]
    const std::vector<int64_t> new_eids = this->request_simplex_indices(PrimitiveType::Edge, 2);
    assert(new_eids.size() == 2);
    std::copy(new_eids.begin(), new_eids.end(), m_split_e.begin());
    const int64_t local_vid = m_mesh.is_ccw(m_operating_tuple) ? 0 : 1;

    // update ee
    {
        // for 2 new edges
        auto ee_new_0 = ee_accessor.index_access().vector_attribute(new_eids[0]);
        auto ee_new_1 = ee_accessor.index_access().vector_attribute(new_eids[1]);
        ee_new_0[local_vid ^ 1] = new_eids[1];
        ee_new_1[local_vid] = new_eids[0];
        if (m_is_self_loop) {
            ee_new_0[local_vid] = new_eids[1];
            ee_new_1[local_vid ^ 1] = new_eids[0];
        } else {
            ee_new_0[local_vid] = m_neighbor_eids[0];
            ee_new_1[local_vid ^ 1] = m_neighbor_eids[1];
            // for neighbor edges
            for (int64_t i = 0; i < 2; i++) {
                if (m_neighbor_eids[i] != -1) {
                    auto ee_neighbor =
                        ee_accessor.index_access().vector_attribute(m_neighbor_eids[i]);
                    auto ev_neighbor =
                        ev_accessor.index_access().vector_attribute(m_neighbor_eids[i]);
                    for (int64_t j = 0; j < 2; j++) {
                        if (ee_neighbor[j] == m_operating_edge_id &&
                            ev_neighbor[j] == m_spine_vids[i]) {
                            ee_neighbor[j] = new_eids[i];
                            break;
                        }
                    }
                }
            }
        }
    }

    // update ev
    {
        // for new edges
        auto ev_new_0 = ev_accessor.index_access().vector_attribute(new_eids[0]);
        auto ev_new_1 = ev_accessor.index_access().vector_attribute(new_eids[1]);
        ev_new_0[local_vid] = m_spine_vids[0];
        ev_new_0[local_vid ^ 1] = v_new;
        ev_new_1[local_vid] = v_new;
        ev_new_1[local_vid ^ 1] = m_spine_vids[1];
    }

    // update ve
    {
        // for new vertex
        ve_accessor.index_access().scalar_attribute(v_new) = new_eids[0];

        // for spine vertices
        ve_accessor.index_access().scalar_attribute(m_spine_vids[0]) = new_eids[0];
        ve_accessor.index_access().scalar_attribute(m_spine_vids[1]) = new_eids[1];
    }
    update_cell_hash();
    delete_simplices();

    // prepare return Tuple
    auto ret_edge = m_mesh.edge_tuple_from_id(new_eids[1]);
    if (m_mesh.id_vertex(ret_edge) != v_new) {
        ret_edge = m_mesh.switch_vertex(ret_edge);
    }

    assert(m_mesh.id_edge(ret_edge) == new_eids[1]);
    assert(m_mesh.id_vertex(ret_edge) == v_new);
    assert(m_mesh.id_vertex(m_mesh.switch_vertex(ret_edge)) == m_spine_vids[1]);

    return ret_edge;
}

void EdgeMesh::EdgeMeshOperationExecutor::update_hash_in_map(EdgeMesh& child_mesh)
{
    // TODO: Implement for multi_mesh in the future
}

void EdgeMesh::EdgeMeshOperationExecutor::collapse_edge()
{
    m_output_tuple = collapse_edge_single_mesh();
    // TODO: Implement for multi_mesh in the future
}


Tuple EdgeMesh::EdgeMeshOperationExecutor::collapse_edge_single_mesh()
{
    // check if the collapse is valid
    if (m_is_self_loop || (m_mesh.is_boundary_vertex(m_operating_tuple) &&
                           m_mesh.is_boundary_vertex(m_mesh.switch_vertex(m_operating_tuple)))) {
        return Tuple();
    }

    simplex_ids_to_delete = get_collapse_simplices_to_delete(m_operating_tuple, m_mesh);

    // update ee
    {
        // for neighbor edges
        for (int64_t i = 0; i < 2; i++) {
            if (m_neighbor_eids[i] != -1) {
                auto ee_neighbor = ee_accessor.index_access().vector_attribute(m_neighbor_eids[i]);
                for (int64_t j = 0; j < 2; j++) {
                    if (ee_neighbor[j] == m_operating_edge_id) {
                        ee_neighbor[j] = m_neighbor_eids[i ^ 1];
                        break;
                    }
                }
            }
        }
    }

    // update ev
    {
        if (m_neighbor_eids[0] != -1) {
            auto ev_neighbor = ev_accessor.index_access().vector_attribute(m_neighbor_eids[0]);
            for (int64_t j = 0; j < 2; j++) {
                if (ev_neighbor[j] == m_spine_vids[0]) {
                    ev_neighbor[j] = m_spine_vids[1];
                }
            }
        }
    }


    // update ve
    {
        ve_accessor.index_access().scalar_attribute(m_spine_vids[1]) = m_neighbor_eids[1];
    }

    update_cell_hash();
    delete_simplices();

    const int64_t ret_eid = m_neighbor_eids[0] == -1 ? m_neighbor_eids[1] : m_neighbor_eids[0];
    Tuple ret_tuple = m_mesh.edge_tuple_from_id(ret_eid);

    if (m_mesh.id_vertex(ret_tuple) != m_spine_vids[1]) {
        ret_tuple = m_mesh.switch_vertex(ret_tuple);
    }
    return ret_tuple;
}

std::vector<int64_t> EdgeMesh::EdgeMeshOperationExecutor::request_simplex_indices(
    const PrimitiveType type,
    int64_t count)
{
    m_mesh.guarantee_more_attributes(type, count);
    return m_mesh.request_simplex_indices(type, count);
}

} // namespace wmtk
