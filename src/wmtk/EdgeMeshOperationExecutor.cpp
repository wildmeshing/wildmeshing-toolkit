
#include "EdgeMeshOperationExecutor.hpp"

namespace wmtk {
// constructor
EdgeMesh::EdgeMeshOperationExecutor::EdgeMeshOperationExecutor(
    EdgeMesh& m,
    const Tuple& operating_tuple,
    Accessor<long>& hash_acc)
    : flag_accessors{{m.get_flag_accessor(PrimitiveType::Vertex), m.get_flag_accessor(PrimitiveType::Edge)}}
    , ee_accessor(m.create_accessor<long>(m.m_ee_handle))
    , ev_accessor(m.create_accessor<long>(m.m_ev_handle))
    , ve_accessor(m.create_accessor<long>(m.m_ve_handle))
    , hash_accessor(hash_acc)
    , m_mesh(m)
    , m_operating_tuple(operating_tuple)

{
    Tuple operating_tuple_switch_vertex = m_mesh.switch_vertex(operating_tuple);
    // store ids of incident vertices
    m_operating_edge_id = m_mesh.id_edge(m_operating_tuple);
    m_spine_vids[0] = m_mesh.id_vertex(m_operating_tuple);
    m_spine_vids[1] = m_mesh.id_vertex(operating_tuple_switch_vertex);

    // update hash on neighborhood
    cell_ids_to_update_hash.emplace_back(m_mesh.id_edge(m_operating_tuple));
    if (m_mesh.is_boundary(m_operating_tuple)) {
        m_neighbor_eids[0] = m_mesh.id_edge(m_mesh.switch_edge(m_operating_tuple));
        cell_ids_to_update_hash.emplace_back(m_neighbor_eids[0]);
    }
    if (m_mesh.is_boundary(operating_tuple_switch_vertex)) {
        m_neighbor_eids[1] = m_mesh.id_edge(m_mesh.switch_edge(operating_tuple_switch_vertex));
        cell_ids_to_update_hash.emplace_back(m_neighbor_eids[1]);
    }
}

void EdgeMesh::EdgeMeshOperationExecutor::delete_simplices()
{
    for (size_t d = 0; d < simplex_ids_to_delete.size(); ++d) {
        for (const long id : simplex_ids_to_delete[d]) {
            flag_accessors[d].index_access().scalar_attribute(id) = 0;
        }
    }
}

void EdgeMesh::EdgeMeshOperationExecutor::update_cell_hash()
{
    m_mesh.update_cell_hashes(cell_ids_to_update_hash, hash_accessor);
}

const std::array<std::vector<long>, 2>
EdgeMesh::EdgeMeshOperationExecutor::get_split_simplices_to_delete(
    const Tuple& tuple,
    const EdgeMesh& m)
{
    std::array<std::vector<long>, 2> ids;
    ids[1].emplace_back(m.id_edge(tuple));
    return ids;
}

const std::array<std::vector<long>, 2>
EdgeMesh::EdgeMeshOperationExecutor::get_collapse_simplices_to_delete(
    const Tuple& tuple,
    const EdgeMesh& m)
{
    std::array<std::vector<long>, 2> ids;
    ids[0].emplace_back(m.id_vertex(tuple));
    ids[1].emplace_back(m.id_edge(tuple));
    return ids;
}

std::vector<Tuple> EdgeMesh::EdgeMeshOperationExecutor::prepare_operating_tuples_for_child_meshes()
    const
{
    // this function is designed as a helper for multi_mesh
    return MultiMeshManager::map_edge_tuple_to_all_children(m_mesh, m_operating_tuple);
}

Tuple EdgeMesh::EdgeMeshOperationExecutor::split_edge()
{
    return split_edge_single_mesh();
    // TODO: Implement for multi_mesh in the future
}

Tuple EdgeMesh::EdgeMeshOperationExecutor::split_edge_single_mesh()
{
    simplex_ids_to_delete = get_split_simplices_to_delete(m_operating_tuple, m_mesh);

    // create new vertex (center)
    std::vector<long> new_vids = this->request_simplex_indices(PrimitiveType::Vertex, 1);
    assert(new_vids.size() == 1);
    const long v_new = new_vids[0];
    // create new edges
    std::vector<long> new_eids = this->request_simplex_indices(PrimitiveType::Edge, 2);
    assert(new_eids.size() == 2);

    long local_vid = m_mesh.is_ccw(m_operating_tuple) ? 0 : 1;

    // update ee
    {
        auto ee_new_0 = ee_accessor.index_access().vector_attribute(new_eids[0]);
        auto ee_new_1 = ee_accessor.index_access().vector_attribute(new_eids[1]);
        ee_new_0[local_vid] = m_neighbor_eids[0];
        ee_new_0[(local_vid + 1) % 2] = new_eids[1];
        ee_new_1[local_vid] = new_eids[0];
        ee_new_1[(local_vid + 1) % 2] = m_neighbor_eids[1];
        for (long i = 0; i < 2; i++) {
            if (m_neighbor_eids[i] != -1) {
                ee_accessor.index_access().vector_attribute(
                    m_neighbor_eids[i])[(local_vid + 1) % 2] = new_eids[i];
            }
        }
    }
    // update ev
    {
        ev_accessor.index_access().scalar_attribute(new_eids[0]) = v_new;
        ev_accessor.index_access().scalar_attribute(new_eids[1]) = v_new;
    }
    // update ve
    {
        ve_accessor.index_access().scalar_attribute(v_new) = new_eids[0];
        ve_accessor.index_access().scalar_attribute(m_spine_vids[0]) = new_eids[0];
        ve_accessor.index_access().scalar_attribute(m_spine_vids[1]) = new_eids[1];
    }
    update_cell_hash();
    delete_simplices();
    return Tuple();
}

void EdgeMesh::EdgeMeshOperationExecutor::update_hash_in_map(EdgeMesh& child_mesh)
{
    // TODO: Implement for multi_mesh in the future
}

Tuple EdgeMesh::EdgeMeshOperationExecutor::collapse_edge()
{
    return collapse_edge_single_mesh();
    // TODO: Implement for multi_mesh in the future
}


Tuple EdgeMesh::EdgeMeshOperationExecutor::collapse_edge_single_mesh()
{
    simplex_ids_to_delete = get_collapse_simplices_to_delete(m_operating_tuple, m_mesh);
    // TODO: Implement this

    update_cell_hash();
    delete_simplices();
    return Tuple();
}

std::vector<long> EdgeMesh::EdgeMeshOperationExecutor::request_simplex_indices(
    const PrimitiveType type,
    long count)
{
    m_mesh.reserve_attributes(type, m_mesh.capacity(type) + count);
    return m_mesh.request_simplex_indices(type, count);
}

} // namespace wmtk
