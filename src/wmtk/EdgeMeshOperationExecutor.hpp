#pragma once
#include <wmtk/operations/edge_mesh/EdgeOperationData.hpp>
#include <wmtk/utils/Logger.hpp>
#include "EdgeMesh.hpp"
#include "Tuple.hpp"
namespace wmtk {
class EdgeMesh::EdgeMeshOperationExecutor : public operations::edge_mesh::EdgeOperationData
{
public:
    EdgeMeshOperationExecutor(EdgeMesh& m, const Tuple& operating_tuple);
    void delete_simplices();

    std::array<attribute::FlagAccessor<EdgeMesh>, 2> flag_accessors;
    attribute::Accessor<int64_t, EdgeMesh> ee_accessor;
    attribute::Accessor<int64_t, EdgeMesh> ev_accessor;
    attribute::Accessor<int64_t, EdgeMesh> ve_accessor;

    /**
     * @brief gather all simplices that are deleted in a split
     *
     * The deleted simplex is the edge itself
     * @return std::array<std::vector<int64_t>, 2> first vector contains the vertex ids, second
     * vector contains the edge ids
     */
    static const std::array<std::vector<int64_t>, 2> get_split_simplices_to_delete(
        const Tuple& tuple,
        const EdgeMesh& m);

    /**
     * @brief gather all simplices that are deleted in a collapse
     *
     * The deleted simplices are the vertex and the edge of the input tuple
     * @return std::array<std::vector<int64_t>, 2> first vector contains the vertex ids, second
     * vector contains the edge ids
     */
    static const std::array<std::vector<int64_t>, 2> get_collapse_simplices_to_delete(
        const Tuple& tuple,
        const EdgeMesh& m);

    const std::array<int64_t, 2>& incident_vids() const { return m_spine_vids; }

    int64_t operating_edge_id() const { return m_operating_edge_id; }

    void split_edge();
    void collapse_edge();
    Tuple split_edge_single_mesh();
    Tuple collapse_edge_single_mesh();

    std::vector<int64_t> request_simplex_indices(const PrimitiveType type, int64_t count);

    EdgeMesh& m_mesh;


private:
};
} // namespace wmtk
