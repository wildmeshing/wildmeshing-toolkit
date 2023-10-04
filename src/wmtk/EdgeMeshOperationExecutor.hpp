#pragma once
#include <wmtk/utils/Logger.hpp>
#include "EdgeMesh.hpp"
#include "SimplicialComplex.hpp"
#include "Tuple.hpp"
namespace wmtk {
class EdgeMesh::EdgeMeshOperationExecutor
{
public:
    EdgeMeshOperationExecutor(EdgeMesh& m, const Tuple& operating_tuple, Accessor<long>& hash_acc);
    void delete_simplices();
    void update_cell_hash();

    std::array<Accessor<char>, 2> flag_accessors;
    Accessor<long> ee_accessor;
    Accessor<long> ev_accessor;
    Accessor<long> ve_accessor;
    Accessor<long>& hash_accessor;

    /**
     * @brief gather all simplices that are deleted in a split
     *
     * The deleted simplex is the edge itself
     * @return std::array<std::vector<long>, 2> first vector contains the vertex ids, second vector
     * contains the edge ids
     */
    static const std::array<std::vector<long>, 2> get_split_simplices_to_delete(
        const Tuple& tuple,
        const EdgeMesh& m);

    /**
     * @brief gather all simplices that are deleted in a collapse
     *
     * The deleted simplices are the vertex and the edge of the input tuple
     * @return std::array<std::vector<long>, 2> first vector contains the vertex ids, second vector
     * contains the edge ids
     */
    static const std::array<std::vector<long>, 2> get_collapse_simplices_to_delete(
        const Tuple& tuple,
        const EdgeMesh& m);

    const std::array<long, 2>& incident_vids() const { return m_spine_vids; }

    long operating_edge_id() const { return m_operating_edge_id; }

    Tuple split_edge();
    Tuple collapse_edge();
    Tuple split_edge_single_mesh();
    Tuple collapse_edge_single_mesh();

    std::vector<long> request_simplex_indices(const PrimitiveType type, long count);

    std::array<std::vector<long>, 2> simplex_ids_to_delete;
    std::vector<long> cell_ids_to_update_hash;
    EdgeMesh& m_mesh;
    Tuple m_operating_tuple;


private:
    std::vector<Tuple> prepare_operating_tuples_for_child_meshes() const;
    void update_hash_in_map(EdgeMesh& child_mesh);
    bool m_is_self_loop = false;
    // common simplicies
    std::array<long, 2> m_spine_vids; // V_A_id, V_B_id;
    std::array<long, 2> m_neighbor_eids = {-1, -1};
    long m_operating_edge_id;
};
} // namespace wmtk
