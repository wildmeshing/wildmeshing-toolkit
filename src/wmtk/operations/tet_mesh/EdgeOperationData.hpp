#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>
#include <wmtk/operations/EdgeOperationData.hpp>

namespace wmtk::operations::tet_mesh {
class EdgeOperationData : public wmtk::operations::EdgeOperationData
{
public:
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
        int64_t tid = -1; // global tid of the ear, -1 if it doesn't exist
        int64_t fid = -1; // global fid of the ear, -1 if it doesn't exist
    };

    /**
     *  Data on the incident tets of the operating edge
     */
    struct IncidentTetData
    {
        int64_t tid = -1;
        std::array<EarTet, 2> ears;
    };

    /**
     * @brief structs for split (to be merge with collapse)
     *
     */

    struct FaceSplitData
    {
        int64_t fid_old = -1;
        int64_t fid_new_1 = -1;
        int64_t fid_new_2 = -1;
        int64_t eid_spine_old = -1;
        int64_t eid_spine_1 = -1;
        int64_t eid_spine_2 = -1;
        int64_t eid_split = -1;
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
        int64_t tid_old = -1;
        int64_t tid_new_1 = -1;
        int64_t tid_new_2 = -1;
        int64_t fid_split = -1;
        int64_t v1;
        int64_t v2;
        int64_t v3;
        int64_t v4;
        int64_t e12;
        int64_t e13;
        int64_t e14;
        int64_t e23;
        int64_t e24;
        int64_t e34;

        EarTet ear_tet_1; // switch edge switch face
        EarTet ear_tet_2; // switch vertex switch edge switch face
        std::array<FaceSplitData, 2> new_face_data;
    };

    struct TetCollapseData
    {
        int64_t tid_old = -1;
        int64_t v1;
        int64_t v2;
        int64_t v3;
        int64_t v4;
        int64_t e12;
        int64_t e13;
        int64_t e14;
        int64_t e23;
        int64_t e24;
        int64_t e34;

        EarTet ear_tet_1; // switch edge switch face
        EarTet ear_tet_2; // switch vertex switch edge switch face

        // the new edge created by merging two ears in a collapse
        int64_t collapse_new_face_id = -1;
    };

    struct FaceCollapseData
    {
    };

    const std::array<int64_t, 2>& incident_vids() const { return m_spine_vids; }

    int64_t operating_edge_id() const { return m_operating_edge_id; }


    std::array<std::vector<int64_t>, 4> simplex_ids_to_delete;
    std::array<std::vector<Tuple>, 4> simplex_tuples_to_delete;
    std::vector<int64_t> cell_ids_to_update_hash;


    // only returns valid tuples for the state before an operation occurred
    std::vector<std::array<Tuple, 2>> ear_edges(const TetMesh& m) const;
    std::vector<std::array<Tuple, 2>> ear_faces(const TetMesh& m) const;
    std::array<Tuple, 2> input_endpoints(const TetMesh& m) const;
    std::vector<Tuple> collapse_merged_ear_edges(const TetMesh& m) const;
    std::vector<Tuple> collapse_merged_ear_faces(const TetMesh& m) const;


protected:
    // common simplices
    std::array<int64_t, 2> m_spine_vids; // two endpoints of the edge
    int64_t m_operating_edge_id;
    int64_t m_operating_face_id;
    int64_t m_operating_tet_id;

    int64_t m_split_new_vid = -1;
    std::array<int64_t, 2> m_split_new_spine_eids;

    // simplices required per-tet
    std::vector<IncidentTetData> m_incident_tet_datas;

    std::vector<TetCollapseData> tet_collapse_data;
    std::vector<TetSplitData> tet_split_data;
};
} // namespace wmtk::operations::tet_mesh
