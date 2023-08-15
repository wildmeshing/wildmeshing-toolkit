#pragma once
#include <wmtk/utils/Logger.hpp>
#include "SimplicialComplex.hpp"
#include "TetMesh.hpp"
#include "Tuple.hpp"
namespace wmtk {
class TetMesh::TetMeshOperationExecutor
{
public:
    TetMeshOperationExecutor(TetMesh& m, const Tuple& operating_tuple);
    void delete_simplices();
    void update_cell_hash();

    std::array<Accessor<char>, 4> flag_accessors;
    Accessor<long> tt_accessor;
    Accessor<long> tf_accessor;
    Accessor<long> te_accessor;
    Accessor<long> tv_accessor;
    Accessor<long> vt_accessor;
    Accessor<long> et_accessor;
    Accessor<long> ft_accessor;
    Accessor<long> hash_accessor;

    /**
     * @brief gather all simplices that are deleted in a split
     *
     * The deleted simplices are the one ring tets AND the one ring faces of the edge AND the edge
     * itself. That is, the open star of the edge.
     */
    static const std::array<std::vector<long>, 4> get_split_simplices_to_delete(
        const Tuple& tuple,
        const TetMesh& m);

    /**
     * @brief gather all simplices that are deleted in a collapse
     *
     * For interior case:
     * The deleted simplices are the one ring tets of the edge AND the one ring faces of the edge
     * AND the edges where one of the endpoint is the tuple vertex and are contained in the deleted
     * faces and the tuple vertex. This is the intersection of the open star of the tuple vertex and
     * the close star of the edge.
     *
     * For boundary case:
     * Same as above.
     */
    static const std::array<std::vector<long>, 4> get_collapse_simplices_to_delete(
        const Tuple& tuple,
        const TetMesh& m);

    const std::array<long, 2>& incident_vids() const { return m_spine_vids; }

    const long operating_edge_id() const { return m_operating_edge_id; }


    Tuple split_edge();

    Tuple collapse_edge();

    std::vector<long> request_simplex_indices(const PrimitiveType type, long count);

    std::array<std::vector<long>, 4> simplex_ids_to_delete;

    TetMesh& m_mesh;
    Tuple m_operating_tuple;


private:
    // common simplices
    std::array<long, 2> m_spine_vids; // two endpoints of the edge
    long m_operating_edge_id;
};
} // namespace wmtk