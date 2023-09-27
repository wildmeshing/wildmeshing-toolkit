
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
    // store ids of edge and incident vertices
    m_operating_edge_id = m_mesh.id_edge(m_operating_tuple);
    m_spine_vids[0] = m_mesh.id_vertex(m_operating_tuple);
    m_spine_vids[1] = m_mesh.id_vertex(m_mesh.switch_vertex(m_operating_tuple));

    const SimplicialComplex edge_closed_star =
        SimplicialComplex::closed_star(m_mesh, Simplex::edge(operating_tuple));

    // get all faces incident to the edge
    for (const Simplex& f : edge_closed_star.get_faces()) {
        // m_incident_face_datas.emplace_back(get_incident_face_data(f.tuple()));
    }

    // update hash on all faces in the two-ring neighborhood
    SimplicialComplex hash_update_region(m);
    for (const Simplex& v : edge_closed_star.get_vertices()) {
        const SimplicialComplex v_closed_star = SimplicialComplex::closed_star(m_mesh, v);
        hash_update_region.unify_with_complex(v_closed_star);
    }
    for (const Simplex& f : hash_update_region.get_faces()) {
        cell_ids_to_update_hash.push_back(m_mesh.id(f));
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

const std::array<std::vector<long>, 3>
EdgeMesh::EdgeMeshOperationExecutor::get_split_simplices_to_delete(
    const Tuple& tuple,
    const EdgeMesh& m)
{
    const SimplicialComplex sc = SimplicialComplex::open_star(m, Simplex::edge(tuple));
    std::array<std::vector<long>, 3> ids;
    for (const Simplex& s : sc.get_simplices()) {
        ids[get_simplex_dimension(s.primitive_type())].emplace_back(m.id(s));
    }

    return ids;
}

const std::array<std::vector<long>, 3>
EdgeMesh::EdgeMeshOperationExecutor::get_collapse_simplices_to_delete(
    const Tuple& tuple,
    const EdgeMesh& m)
{
    const SimplicialComplex vertex_open_star =
        SimplicialComplex::open_star(m, Simplex::vertex(tuple));
    const SimplicialComplex edge_closed_star =
        SimplicialComplex::closed_star(m, Simplex::edge(tuple));

    const SimplicialComplex sc =
        SimplicialComplex::get_intersection(vertex_open_star, edge_closed_star);

    std::array<std::vector<long>, 3> ids;
    for (const Simplex& s : sc.get_simplices()) {
        ids[get_simplex_dimension(s.primitive_type())].emplace_back(m.id(s));
    }

    return ids;
}

std::vector<std::vector<Tuple>>
EdgeMesh::EdgeMeshOperationExecutor::prepare_operating_tuples_for_child_meshes() const
{
    return std::vector<std::vector<Tuple>>();
}

Tuple EdgeMesh::EdgeMeshOperationExecutor::split_edge()
{
    return Tuple();
}

Tuple EdgeMesh::EdgeMeshOperationExecutor::split_edge_single_mesh()
{
    return Tuple();
    // return m_mesh.with_different_cid(m_operating_tuple, m_incident_face_datas[0].split_f0);
}

void EdgeMesh::EdgeMeshOperationExecutor::update_hash_in_map(EdgeMesh& child_mesh)
{
    long child_id = child_mesh.multi_mesh_manager.child_id();
    auto child_hash_accessor = child_mesh.get_cell_hash_accessor();

    for (auto parent_cell_id : cell_ids_to_update_hash) {
        auto [t_parent_old, t_child_old] = MultiMeshManager::read_tuple_map_attribute(
            m_mesh.multi_mesh_manager.map_to_child_handles[child_id],
            m_mesh,
            m_mesh.tuple_from_id(m_mesh.top_simplex_type(), parent_cell_id));

        long parent_cell_hash = m_mesh.get_cell_hash_slow(parent_cell_id);
        Tuple t_parent_new = t_parent_old.with_updated_hash(parent_cell_hash);

        if (t_child_old.is_null()) {
            MultiMeshManager::write_tuple_map_attribute(
                m_mesh.multi_mesh_manager.map_to_child_handles[child_id],
                m_mesh,
                t_parent_new,
                Tuple());
        } else {
            long child_cell_hash =
                child_hash_accessor.index_access().const_scalar_attribute(t_child_old.m_global_cid);
            Tuple t_child_new = t_child_old.with_updated_hash(child_cell_hash);
            MultiMeshManager::write_tuple_map_attribute(
                m_mesh.multi_mesh_manager.map_to_child_handles[child_id],
                m_mesh,
                t_parent_new,
                t_child_new);

            MultiMeshManager::write_tuple_map_attribute(
                child_mesh.multi_mesh_manager.map_to_parent_handle,
                child_mesh,
                t_child_new,
                t_parent_new);
        }
    }
}

Tuple EdgeMesh::EdgeMeshOperationExecutor::collapse_edge()
{
    return Tuple();
}


Tuple EdgeMesh::EdgeMeshOperationExecutor::collapse_edge_single_mesh()
{
    return Tuple();

    // return a ccw tuple from left ear if it exists, otherwise return a ccw tuple from right ear
    // return m_mesh.tuple_from_id(PrimitiveType::Vertex, v1);
}

std::vector<long> EdgeMesh::EdgeMeshOperationExecutor::request_simplex_indices(
    const PrimitiveType type,
    long count)
{
    return std::vector<long>();
}

} // namespace wmtk
