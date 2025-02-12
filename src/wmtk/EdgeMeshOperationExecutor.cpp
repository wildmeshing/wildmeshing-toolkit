
#include "EdgeMeshOperationExecutor.hpp"
#include <wmtk/operations/internal/SplitAlternateFacetData.hpp>

namespace wmtk {
// constructor
EdgeMesh::EdgeMeshOperationExecutor::EdgeMeshOperationExecutor(
    EdgeMesh& m,
    const Tuple& operating_tuple)
    : flag_accessors{{m.get_flag_accessor(PrimitiveType::Vertex), m.get_flag_accessor(PrimitiveType::Edge)}}
    , ee_accessor(m.create_accessor<int64_t>(m.m_ee_handle))
    , ev_accessor(m.create_accessor<int64_t>(m.m_ev_handle))
    , ve_accessor(m.create_accessor<int64_t>(m.m_ve_handle))
    , m_mesh(m)
{
    m_operating_tuple = operating_tuple;
    Tuple operating_tuple_switch_vertex = m_mesh.switch_vertex(operating_tuple);
    // store ids of incident vertices
    m_operating_edge_id = m_mesh.id_edge(m_operating_tuple);
    m_spine_vids[0] = m_mesh.id_vertex(m_operating_tuple);
    m_spine_vids[1] = m_mesh.id_vertex(operating_tuple_switch_vertex);

    // update hash on neighborhood
    if (!m_mesh.is_boundary_vertex(m_operating_tuple)) {
        m_neighbor_eids[0] = m_mesh.id_edge(m_mesh.switch_edge(m_operating_tuple));
    }
    if (!m_mesh.is_boundary_vertex(operating_tuple_switch_vertex)) {
        m_neighbor_eids[1] = m_mesh.id_edge(m_mesh.switch_edge(operating_tuple_switch_vertex));
    }


    if (m_neighbor_eids[0] == m_neighbor_eids[1] && m_neighbor_eids[0] == m_operating_edge_id) {
        m_is_self_loop = true;
    }
}

void EdgeMesh::EdgeMeshOperationExecutor::delete_simplices()
{
    for (size_t d = 0; d < simplex_ids_to_delete.size(); ++d) {
        for (const int64_t id : simplex_ids_to_delete[d]) {
            flag_accessors[d].index_access().deactivate(id) ;
        }
    }
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
    if (m.is_free()) {
        ids[0].emplace_back(m.id_vertex(m.switch_tuple(tuple, PrimitiveType::Vertex)));
    }
    return ids;
}


void EdgeMesh::EdgeMeshOperationExecutor::split_edge()
{
    set_split();
    m_output_tuple = split_edge_single_mesh();
    // TODO: Implement for multi_mesh in the future
}

Tuple EdgeMesh::EdgeMeshOperationExecutor::split_edge_single_mesh()
{
    simplex_ids_to_delete = get_split_simplices_to_delete(m_operating_tuple, m_mesh);

    // create new edges (facets)
    // m_split_e[i] is connect to m_neighbor_eids[i] and m_spine_vids[i]
    const auto& data = split_facet_data().add_facet(m_mesh, m_operating_tuple);
    m_split_e = data.new_facet_indices;

    if (m_mesh.is_free()) {
        const std::vector<int64_t> new_vids =
            this->request_simplex_indices(PrimitiveType::Vertex, 2);
        assert(new_vids.size() == 2);
        std::copy(new_vids.begin(), new_vids.end(), m_free_split_v.begin());
        const int64_t v_new = new_vids[0];
        m_split_v = -1;
    } else {
        // create new vertex
        const std::vector<int64_t> new_vids =
            this->request_simplex_indices(PrimitiveType::Vertex, 1);
        assert(new_vids.size() == 1);
        const int64_t v_new = new_vids[0];
        m_split_v = v_new;
    }
    const int64_t local_vid = m_mesh.is_ccw(m_operating_tuple) ? 0 : 1;

    // update ee
    if (m_mesh.is_free()) {
    } else {
        // for 2 new edges
        auto ee_new_0 = ee_accessor.index_access().vector_attribute(m_split_e[0]);
        auto ee_new_1 = ee_accessor.index_access().vector_attribute(m_split_e[1]);
        ee_new_0[local_vid ^ 1] = m_split_e[1];
        ee_new_1[local_vid] = m_split_e[0];
        if (m_is_self_loop) {
            ee_new_0[local_vid] = m_split_e[1];
            ee_new_1[local_vid ^ 1] = m_split_e[0];
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
                            ee_neighbor[j] = m_split_e[i];
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
        auto ev_new_0 = ev_accessor.index_access().vector_attribute(m_split_e[0]);
        auto ev_new_1 = ev_accessor.index_access().vector_attribute(m_split_e[1]);
        ev_new_0[local_vid] = m_spine_vids[0];
        if (m_mesh.is_free()) {
            ev_new_0[local_vid ^ 1] = m_free_split_v[0];
            ev_new_1[local_vid] = m_free_split_v[1];
        } else {
            ev_new_0[local_vid ^ 1] = m_split_v;
            ev_new_1[local_vid] = m_split_v;
        }
        ev_new_1[local_vid ^ 1] = m_spine_vids[1];
    }

    // update ve
    {
        // for new vertex
        if (m_mesh.is_free()) {
            ve_accessor.index_access().scalar_attribute(m_free_split_v[0]) = m_split_e[0];
            ve_accessor.index_access().scalar_attribute(m_free_split_v[1]) = m_split_e[1];
        } else {
            ve_accessor.index_access().scalar_attribute(m_split_v) = m_split_e[0];
        }

        // for spine vertices
        ve_accessor.index_access().scalar_attribute(m_spine_vids[0]) = m_split_e[0];
        ve_accessor.index_access().scalar_attribute(m_spine_vids[1]) = m_split_e[1];
    }
    delete_simplices();

    // prepare return Tuple
    auto ret_edge = m_mesh.edge_tuple_from_id(m_split_e[1]);

    // if the mesh is free we don't care about which edge is returned
    if (!m_mesh.is_free()) {
        if (m_mesh.id_vertex(ret_edge) != m_split_v) {
            ret_edge = m_mesh.switch_vertex(ret_edge);

            assert(m_mesh.id_edge(ret_edge) == m_split_e[1]);
            assert(m_mesh.id_vertex(ret_edge) == m_split_v);
            assert(m_mesh.id_vertex(m_mesh.switch_vertex(ret_edge)) == m_spine_vids[1]);
        }
    }

    return ret_edge;
}


void EdgeMesh::EdgeMeshOperationExecutor::collapse_edge()
{
    set_collapse();
    m_output_tuple = collapse_edge_single_mesh();
    // TODO: Implement for multi_mesh in the future
}


Tuple EdgeMesh::EdgeMeshOperationExecutor::collapse_edge_single_mesh()
{
    if (m_mesh.is_free()) {
        simplex_ids_to_delete = get_collapse_simplices_to_delete(m_operating_tuple, m_mesh);
        delete_simplices();
        return Tuple();
        ;
    }
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
        ve_accessor.index_access().scalar_attribute(m_spine_vids[1]) =
            (m_neighbor_eids[1] != -1) ? m_neighbor_eids[1] : m_neighbor_eids[0];
    }

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
