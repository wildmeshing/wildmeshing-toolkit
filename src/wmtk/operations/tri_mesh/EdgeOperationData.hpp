#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>

namespace wmtk::operations::tri_mesh {
struct EdgeOperationData
{
    EdgeOperationData() = default;
    EdgeOperationData(const EdgeOperationData&) = default;
    EdgeOperationData(EdgeOperationData&&) = default;

    EdgeOperationData& operator=(const EdgeOperationData&) = default;
    EdgeOperationData& operator=(EdgeOperationData&&) = default;
    //           C
    //         /  \ .
    //    F1  /    \  F2
    //       /      \ .
    //      /        \ .
    //     A----------B
    //      \        /
    //       \      /
    //    F1' \    / F2'
    //         \  /
    //          C'
    // the neighbors are stored in the order of A, B, C, D if they exist
    // vid, ear fid (-1 if it doesn't exit), ear eid

    /**
     * An ear is a face that is adjacent to a face that is incident to the edge on which the
     * operation is performed. In other words, the ears are the neighboring faces to the ones
     * that will be deleted by the operation.
     */
    struct EarFace
    {
        long fid = -1; // global fid of the ear, -1 if it doesn't exist
        long eid = -1; // global eid of the ear, -1 if it doesn't exist
    };

    /**
     * Data on the incident face relevant for performing operations.
     */
    struct IncidentFaceData
    {
        long opposite_vid = -1; // opposing vid
        long fid = -1; // the face that will be deleted
        std::array<long, 2> split_f = std::array<long, 2>{{-1, -1}};
        Tuple local_operating_tuple; // the copy of edge m_operating_tuple in face(fid)
        std::array<EarFace, 2> ears; // ear
    };

    std::vector<IncidentFaceData>& incident_face_datas() { return m_incident_face_datas; }

    const std::array<long, 2>& incident_vids() const { return m_spine_vids; }

    long operating_edge_id() const { return m_operating_edge_id; }


    std::array<std::vector<long>, 3> simplex_ids_to_delete;
    std::vector<long> cell_ids_to_update_hash;

    Tuple m_operating_tuple;

    Tuple m_output_tuple; // reference tuple for either operation

    // common simplicies
    std::array<long, 2> m_spine_vids; // V_A_id, V_B_id;
    long m_operating_edge_id;

    // simplices required per-face
    std::vector<IncidentFaceData> m_incident_face_datas;
};
} // namespace wmtk::operations::tri_mesh
