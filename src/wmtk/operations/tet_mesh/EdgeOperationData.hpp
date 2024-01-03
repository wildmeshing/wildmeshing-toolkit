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
    //     \  tid0   /   |   \   tid1  /
    //      \     -_/fid0|fid1\_-     /
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


    /*
               v2
               /\\
        ear1  /  \ \   ear2
             /    \  \
            /      \   \
           /        \    \
          /          \     \
         /            \     _\ v3
        /______________\_ -
       v0     e01       v1
    */

    struct FaceSplitData
    {
        int64_t fid_old = -1;
        std::array<int64_t, 2> fid_new = std::array<int64_t, 2>{{-1, -1}};
        int64_t eid_spine_old = -1;
        std::array<int64_t, 2> eid_spine_new = std::array<int64_t, 2>{{-1, -1}};
        int64_t eid_split = -1;
    };

    struct FaceCollapseData
    {
    };

    /**
     *  Data on the incident tets of the operating edge
     */
    struct IncidentTetData
    {
        // merging split data and collapse data
        int64_t tid = -1;

        std::array<int64_t, 2> split_t = std::array<int64_t, 2>{{-1, -1}}; // tid_new_1/2
        int64_t split_f = -1; // fid_split

        int64_t v0;
        int64_t v1;
        int64_t v2;
        int64_t v3;
        int64_t e01;
        int64_t e02;
        int64_t e03;
        int64_t e12;
        int64_t e13;
        int64_t e23;

        std::array<EarTet, 2> ears; // ear_tet_1/2

        std::array<FaceSplitData, 2> new_face_data;

        // should = split_f, new rib face for split or face merging two ears by collapse
        int64_t new_face_id = -1;
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

    std::vector<IncidentTetData> incident_tet_datas() const { return m_incident_tet_datas; }


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

    std::vector<IncidentTetData> tet_collapse_data;
    std::vector<IncidentTetData> tet_split_data;
};
} // namespace wmtk::operations::tet_mesh
