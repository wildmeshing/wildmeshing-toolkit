
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
        cell_ids_to_update_hash.emplace_back(m_mesh.id_edge(m_mesh.switch_edge(m_operating_tuple)));
    }
    if (m_mesh.is_boundary(operating_tuple_switch_vertex)) {
        cell_ids_to_update_hash.emplace_back(
            m_mesh.id_edge(m_mesh.switch_edge(operating_tuple_switch_vertex)));
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
    // TODO: Implement this
    std::array<std::vector<long>, 2> ids;
    return ids;
}

const std::array<std::vector<long>, 2>
EdgeMesh::EdgeMeshOperationExecutor::get_collapse_simplices_to_delete(
    const Tuple& tuple,
    const EdgeMesh& m)
{
    // TODO: Implement this
    std::array<std::vector<long>, 2> ids;
    return ids;
}

std::vector<std::vector<Tuple>>
EdgeMesh::EdgeMeshOperationExecutor::prepare_operating_tuples_for_child_meshes() const
{
    return std::vector<std::vector<Tuple>>();
}

Tuple EdgeMesh::EdgeMeshOperationExecutor::split_edge()
{
    return split_edge_single_mesh();
    // TODO: Implement for multi_mesh in the future
}

Tuple EdgeMesh::EdgeMeshOperationExecutor::split_edge_single_mesh()
{
    // TODO: Implement this
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
    // TODO: Implement this
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
