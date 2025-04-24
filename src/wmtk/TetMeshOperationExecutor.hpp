#pragma once
#include <wmtk/operations/tet_mesh/EdgeOperationData.hpp>
#include <wmtk/utils/Logger.hpp>
#include "TetMesh.hpp"
#include "Tuple.hpp"
namespace wmtk {
class TetMesh::TetMeshOperationExecutor : public operations::tet_mesh::EdgeOperationData
{
public:
    TetMeshOperationExecutor(TetMesh& m, const Tuple& operating_tuple);
    void delete_simplices();

    std::array<attribute::FlagAccessor<TetMesh>, 4> flag_accessors;
    attribute::Accessor<int64_t,TetMesh>& tt_accessor;
    attribute::Accessor<int64_t,TetMesh>& tf_accessor;
    attribute::Accessor<int64_t,TetMesh>& te_accessor;
    attribute::Accessor<int64_t,TetMesh>& tv_accessor;
    attribute::Accessor<int64_t,TetMesh>& vt_accessor;
    attribute::Accessor<int64_t,TetMesh>& et_accessor;
    attribute::Accessor<int64_t,TetMesh>& ft_accessor;


    /**
     * @brief gather all simplices that are deleted in a split
     *
     * The deleted simplices are the one ring tets AND the one ring faces of the edge AND the edge
     * itself. That is, the open star of the edge.
     */
    static const std::array<std::vector<int64_t>, 4> get_split_simplices_to_delete(
        const Tuple& tuple,
        const TetMesh& m);

    /**
     * @brief gather all simplices that are deleted in a collapse
     *
     * For interior case:
     * The deleted simplices are the one ring tets of the edge AND the one ring faces of the edge
     * AND the edges and faces where one of the endpoint is the tuple vertex and are contained in
     * the deleted faces and the tuple vertex. This is the intersection of the open star of the
     * tuple vertex and the close star of the edge.
     *
     * For boundary case:
     * Same as above.
     *
     * @return a pair of vector of ids (int64_t) and tuples(tuple)
     */
    static const std::array<std::vector<int64_t>, 4> get_collapse_simplices_to_delete(
        const Tuple& tuple,
        const TetMesh& m);

    void update_ear_connectivity(
        const int64_t ear_tid,
        const int64_t new_tid,
        const int64_t old_tid,
        const int64_t common_fid);


    /*

       */
    /**
     * @brief split edge v1-v2
     *
     *            v4
     *            /\\
     *     ear1  /| \ \   ear2
     *          / |  \  \
     *         /  |   \   \
     *        /   |    \    \
     *       /    |     \     \
     *      /     |      \     _\ v3
     *     /______|_______\_ -
     *    v1     v_new      v2
     *
     *   input: tuple(v1, v1-v2, v1-v2-v4, v1-v2-v4-v3) (vertex, edge, face, tet)
     *
     * This function will return the tuple that has: the same vertex as the input, a new edge
     * along the input edge, a new face on the input face, and a new tet with is half of the input
     * tet. In the illustration it will return Tuple(v_new, v_new-v2, v_new-v2-v4, v_new-v2-v4-v3)
     *
     */
    void split_edge();

    /**
     * @brief split edge v1-v2
     *
     *
     *     //  5 --------- 4 ---------- 6
     *          \  \      / \\        /
     *           \      \/   \ \     /
     *            \     /    \\  \  /
     *             \   /       \  \\ 3
     *               1 --------- 2/      tuple edge 1-2
     *
     * input: tuple(v1, v1-v2, v1-v2-v4, v1-v2-v4-v3)
     *
     * If tet 2-3-4-6 exists, return Tuple(v2, v2-v4, v2-v4-v3 v2-v4-v3-v6),
     * otherwise return Tuple(v2, v2-v4, v2-v4-v3, v2-v4-v3-v5). Must exist a valid return (check by
     * link condition user level? *should return a invalid tuple if no ears?*).
     *
     */
    void collapse_edge();

    std::vector<int64_t> request_simplex_indices(const PrimitiveType type, int64_t count);


    TetMesh& m_mesh;

private:
    // IncidentTetData get_incident_tet_data(Tuple t);


public:
    /**
     * @brief Get the incident tets and faces for an edge tuple
     *
     * @param edge tuple t (should be m_operating tuple in the operation class)
     * @return std::array<std::vector<Tuple>, 2> array[0] for incident_tets, array[1] for
     * incident_faces. incident_tets.size() == incident_faces.size() if cycle case;
     * incident_tets.size() + 1 == incident_faces.size() if boundary case. incident_tets[i] has face
     * incident_face[(i+incident_faces.size()-1) % incident_faces.size()] and incident_face[i]. The
     * face and tet iterating direction follows the input tuple.
     */

    // TODO: change to i and i+1 mod size convention
    std::tuple<std::vector<Tuple>, std::vector<Tuple>> get_incident_tets_and_faces(Tuple t);
};
} // namespace wmtk
