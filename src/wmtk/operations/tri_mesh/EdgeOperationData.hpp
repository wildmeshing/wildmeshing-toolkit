#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>
#include <wmtk/operations/EdgeOperationData.hpp>

namespace wmtk::operations::tri_mesh {
class EdgeOperationData : public wmtk::operations::EdgeOperationData
{
public:
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
    struct EarData
    {
        int64_t fid = -1; // global fid of the ear, -1 if it doesn't exist
        int64_t eid = -1; // global eid of the ear, -1 if it doesn't exist
    };

    /**
     * Data on the incident face relevant for performing operations.
     */
    struct IncidentFaceData
    {
        // vid of the vertex opposite of the input edge with respect to the input face
        int64_t opposite_vid = -1;
        // the face id from before the operation
        int64_t fid = -1;
        // the fids of the split edge - first one has vertex A, second one has vertex B
        std::array<int64_t, 2> split_f = std::array<int64_t, 2>{{-1, -1}};

        // the copy of edge m_operating_tuple in face(fid)
        Tuple local_operating_tuple;

        // the ear data (i.e FID and EID of the edge/face across the edge.
        // first face/edge include A, second includes B
        std::array<EarData, 2> ears;

        // the new edge created by split/collapse
        // for collapse: merging two ears into this edge
        // for split: new rib edge
        int64_t new_edge_id = -1;

        int64_t merged_edge_fid = -1;
    };

    const std::vector<IncidentFaceData>& incident_face_datas() const
    {
        return m_incident_face_datas;
    }

    const std::array<int64_t, 2>& incident_vids() const { return m_spine_vids; }

    int64_t operating_edge_id() const { return m_operating_edge_id; }

    // only returns valid tuples for the state before an operation occurred
    std::vector<std::array<Tuple, 2>> ear_edges(const TriMesh& m) const;
    std::array<Tuple, 2> input_endpoints(const TriMesh& m) const;
    std::vector<Tuple> collapse_merged_ear_edges(const TriMesh& m) const;

    std::vector<Tuple> split_new_rib_edges(const TriMesh&) const;
    std::vector<Tuple> input_faces(const TriMesh&) const;
    std::array<Tuple, 2> split_output_edges(const TriMesh&) const;
    std::vector<std::array<Tuple, 2>> split_output_faces(const TriMesh&) const;


    std::array<std::vector<int64_t>, 3> simplex_ids_to_delete;

    // for multimesh we need to know which global ids are modified to trigger
    // for every simplex dimension (We have 3 in trimesh):
    // a list of [simplex index, {all versions of that simplex}]
#if defined(WMTK_ENABLE_MULTIMESH)
    std::vector<std::vector<std::tuple<int64_t, std::vector<Tuple>>>>
        global_simplex_ids_with_potentially_modified_hashes;
#endif


    // common simplicies
    int64_t spine_eid = -1;
    int64_t m_operating_edge_id = -1;

    // simplices required per-face
    std::vector<IncidentFaceData> m_incident_face_datas;

    std::array<int64_t, 2> split_spine_eids = std::array<int64_t, 2>{{-1, -1}};
    int64_t split_new_vid = -1;

    bool is_collapse = false;
};
} // namespace wmtk::operations::tri_mesh
