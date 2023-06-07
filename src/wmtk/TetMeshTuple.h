#pragma once
#include <limits>
#include <optional>
#include <tuple>


namespace wmtk {
class TetMesh;
// Cell TetMeshTuple Navigator
/**
 * @brief a TetMeshTuple refers to a global vid and a global tet id, and a local edge id and local
 * face id
 *
 */
class TetMeshTuple
{
    size_t m_global_vid = std::numeric_limits<size_t>::max();
    size_t m_local_eid = std::numeric_limits<size_t>::max();
    size_t m_local_fid = std::numeric_limits<size_t>::max();
    size_t m_global_tid = std::numeric_limits<size_t>::max();

    int m_hash = 0;

private:
    /**
     * Construct a new TetMeshTuple object with global vertex/tetra index and local edge/face index
     *
     * @param vid vertex id (global)
     * @param eid edge id (local)
     * @param fid face id (local)
     * @param tid tetra id (global)
     * @param ts hash associated with tid
     */
    TetMeshTuple(const TetMesh& m, size_t vid, size_t local_eid, size_t local_fid, size_t tid);

public:
    TetMeshTuple() {}

    friend TetMesh;

    /**
     * Check if the current tuple is already invalid (removed during editing).
     *
     * @param m TetMesh where the tuple belongs.
     * @return if not removed and the tuple is up to date with respect to the connectivity. And
     * the tet id can't be -1
     */
    bool is_valid(const TetMesh& m) const;
    /**
     * Check if the current tuple the refers to an edge is on the boundary
     *
     * @param m TetMesh where the tuple belongs.
     * @return if the edge is at the mesh boundary
     */
    bool is_boundary_edge(const TetMesh& m) const;
    /**
     * Check if the current tuple the refers to a face is on the boundary
     *
     * @param m TetMesh where the tuple belongs.
     * @return if the edge is at the mesh boundary
     */
    bool is_boundary_face(const TetMesh& m) const;

    /**
     * @brief prints the tuple
     *
     */
    void print_info() const;

    /**
     * @brief prints additional information
     *
     * @param m mesh
     */
    void print_info(const TetMesh& m) const;

    /**
     * returns global vertex id.
     * @param m TetMesh where the tuple belongs.
     * @return size_t
     */
    size_t vid(const TetMesh& m) const;

    /**
     * returns a global unique edge id
     *
     * @param m TetMesh where the tuple belongs.
     * @return size_t
     * @note The global id may not be consecutive. The edges are undirected and different tetra
     * share the same edge.
     */
    size_t eid(const TetMesh& m) const;

    /**
     * returns a global unique face id
     *
     * @param m TetMesh where the tuple belongs.
     * @return size_t
     * @note The global id may not be consecutive. The face are undirected.
     */
    size_t fid(const TetMesh& m) const;

    /**
     * returns global tetra id.

     * @param m TetMesh where the tuple belongs.
     * @return size_t
     */
    size_t tid(const TetMesh& m) const;

    /**
     * Switch operation.
     *
     * @param m the mesh the TetMeshTuple is in
     * @return another TetMeshTuple that share the same tetra, face, edge, but different vertex.
     */
    TetMeshTuple switch_vertex(const TetMesh& m) const;
    /**
     * Switch operation.
     *
     * @param m the mesh the TetMeshTuple is in
     * @return another TetMeshTuple that share the same tetra, face, vertex, but different edge.
     */
    TetMeshTuple switch_edge(const TetMesh& m) const;
    /**
     * Switch operation.
     *
     * @param m the mesh the TetMeshTuple is in
     * @return another TetMeshTuple that share the same tetra, edge, vertex, but different edge.
     */
    TetMeshTuple switch_face(const TetMesh& m) const;

    /**
     * Switch operation for the adjacent tetra.
     *
     * @param m Mesh
     * @return TetMeshTuple for the face-adjacent tetra, sharing same face, edge, and vertex.
     * @return nullopt if the TetMeshTuple is the switch goes off the boundary.
     */
    std::optional<TetMeshTuple> switch_tetrahedron(const TetMesh& m) const;


    ////testing code
    /**
     * @brief check TetMeshTuple validity and connectivity validity
     */
    void check_validity(const TetMesh& m) const;
    friend bool operator==(const TetMeshTuple& a, const TetMeshTuple& t)
    {
        return (
            std::tie(a.m_global_vid, a.m_local_eid, a.m_local_fid, a.m_global_tid, a.m_hash) ==
            std::tie(t.m_global_vid, t.m_local_eid, t.m_local_fid, t.m_global_tid, t.m_hash));
    }
    friend bool operator<(const TetMeshTuple& a, const TetMeshTuple& t)
    {
        return (
            std::tie(a.m_global_vid, a.m_local_eid, a.m_local_fid, a.m_global_tid, a.m_hash) <
            std::tie(t.m_global_vid, t.m_local_eid, t.m_local_fid, t.m_global_tid, t.m_hash));
    }
};
} // namespace wmtk
