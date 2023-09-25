#pragma once
#include <wmtk/utils/Logger.hpp>
#include "SimplicialComplex.hpp"
#include "TetMesh.hpp"
#include "Tuple.hpp"
namespace wmtk {
class TetMesh::TetMeshOperationExecutor
{
public:
    TetMeshOperationExecutor(TetMesh& m, const Tuple& operating_tuple, Accessor<long>& hash_acc);
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
    Accessor<long>& hash_accessor;

    //
    // E --------------- C --------------- F
    //   \-_           / | \           _-/
    //    \  EarTet   /  |  \   EarTet  /
    //     \  tid1   /   |   \   tid2  /
    //      \     -_/fid1|fid2\_-     /
    //       \     / --_ | _-- \     /
    //        \   /  __- D -__  \   /
    //         \ /_--         --_\ /
    //         A ================= B
    //            operating edge
    //

    /**
     * An EarTet is a neighbor of a tet to be deleted in the split/collapse operation
     *
     */
    struct EarTet
    {
        long tid = -1; // global tid of the ear, -1 if it doesn't exist
        long fid = -1; // global fid of the ear, -1 if it doesn't exist
    };

    /**
     *  Data on the incident tets of the operating edge
     */
    struct IncidentTetData
    {
        long tid = -1;
        std::array<EarTet, 2> ears;
    };

    /**
     * @brief structs for split (to be merge with collapse)
     *
     */

    struct FaceSplitData
    {
        long fid_old = -1;
        long fid_new_1 = -1;
        long fid_new_2 = -1;
        long eid_spine_old = -1;
        long eid_spine_1 = -1;
        long eid_spine_2 = -1;
        long eid_split = -1;
    };

    /*
               v3
               /\\
        ear1  /  \ \   ear2
             /    \  \
            /      \   \
           /        \    \
          /          \     \
         /            \     _\ v4
        /______________\_ -
       v1     e12       v2
    */

    struct TetSplitData
    {
        long tid_old = -1;
        long tid_new_1 = -1;
        long tid_new_2 = -1;
        long fid_split = -1;
        long v1;
        long v2;
        long v3;
        long v4;
        long e12;
        long e13;
        long e14;
        long e23;
        long e24;
        long e34;

        EarTet ear_tet_1; // switch edge switch face
        EarTet ear_tet_2; // switch vertex switch edge switch face
        std::array<FaceSplitData, 2> new_face_data;
    };

    struct TetCollapseData
    {
        long tid_old = -1;
        long v1;
        long v2;
        long v3;
        long v4;
        long e12;
        long e13;
        long e14;
        long e23;
        long e24;
        long e34;

        EarTet ear_tet_1; // switch edge switch face
        EarTet ear_tet_2; // switch vertex switch edge switch face
    };


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
     * AND the edges and faces where one of the endpoint is the tuple vertex and are contained in
     * the deleted faces and the tuple vertex. This is the intersection of the open star of the
     * tuple vertex and the close star of the edge.
     *
     * For boundary case:
     * Same as above.
     */
    static const std::array<std::vector<long>, 4> get_collapse_simplices_to_delete(
        const Tuple& tuple,
        const TetMesh& m);

    void update_ear_connectivity(
        const long ear_tid,
        const long new_tid,
        const long old_tid,
        const long common_fid);

    const std::array<long, 2>& incident_vids() const { return m_spine_vids; }

    const long operating_edge_id() const { return m_operating_edge_id; }


    /*

       */
    /**
     * @brief split edge v1-v2
     *
     *            v3
     *            /\\
     *     ear1  /| \ \   ear2
     *          / |  \  \
     *         /  |   \   \
     *        /   |    \    \
     *       /    |     \     \
     *      /     |      \     _\ v4
     *     /______|_______\_ -
     *    v1     v_new      v2
     *
     *   input: tuple(v1, v1-v2, v1-v2-v3, v1-v2-v3-v4)
     *
     * @return Tuple(v1, v1-v_new, v1-v_new-v3, v1-v_new-v3-v4)
     */
    Tuple split_edge();


    //  5 --------- 3 ---------- 6
    //   \  \      / \\        /
    //    \      \/   \ \     /
    //     \     /    \\  \  /
    //      \   /       \  \\ 4
    //        1 --------- 2/      tuple edge 1-2
    //
    /**
     * @brief split edge v1-v2, input: tuple(v1, v1-v2, either face, v1-v2-v3-v4)
     *
     * @return If tet 2-3-4-6 exists, return Tuple(v2, v2-v3, v2-v3-v4, v2-v3-v4-v6),
     * otherwise return Tuple(v2, v2-v3, v2-v3-v4, v2-v3-v4-v5). Must exist a valid return.
     */
    Tuple collapse_edge();

    std::vector<long> request_simplex_indices(const PrimitiveType type, long count);

    std::array<std::vector<long>, 4> simplex_ids_to_delete;
    std::vector<long> cell_ids_to_update_hash;

    TetMesh& m_mesh;
    Tuple m_operating_tuple;


private:
    // common simplices
    std::array<long, 2> m_spine_vids; // two endpoints of the edge
    long m_operating_edge_id;
    long m_operating_face_id;
    long m_operating_tet_id;

    // simplices required per-tet
    std::vector<IncidentTetData> m_incident_tet_datas;

    IncidentTetData get_incident_tet_data(Tuple t);


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
    std::array<std::vector<Tuple>, 2> get_incident_tets_and_faces(Tuple t);
};
} // namespace wmtk
