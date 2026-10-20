#pragma once

#include "SurfaceHistory.hpp"

#include <array>
#include <cstdint>
#include <set>
#include <vector>

namespace wmtk::history {

/**
 * @brief A triangle complex driven by the edits of a SurfaceHistory.
 *
 * Starts from the history's snapshot and applies edits one at a time, with the semantics the
 * meshes give their operations (see EditKind), so after the k-th edit it holds exactly the
 * recording mesh's state after its k-th committed operation: the same vertex and face ids, the
 * same corner order in every face, the same positions.
 */
class Replay
{
public:
    explicit Replay(const SurfaceHistory& h);

    void apply(const Edit& e);
    /// Applies every edit of `h`, which must be the history this was built from.
    void apply_all(const SurfaceHistory& h);

    const std::vector<Eigen::Vector3d>& vertices() const { return m_vertices; }
    const std::vector<uint8_t>& vertex_alive() const { return m_vertex_alive; }
    const std::vector<std::array<uint64_t, 3>>& faces() const { return m_faces; }
    const std::vector<uint8_t>& face_alive() const { return m_face_alive; }
    /// The live faces at vertex `v`, sorted.
    const std::vector<uint64_t>& faces_of(uint64_t v) const { return m_vertex_faces[v]; }
    /// The feature edges (see SurfaceHistory::feature_edges), smaller id first.
    std::set<std::array<uint64_t, 2>> feature_edges() const;
    bool is_feature(uint64_t a, uint64_t b) const;

private:
    void split_edge(const Edit& e);
    void split_face(const Edit& e);
    void collapse_edge(const Edit& e);
    void swap_edge(const Edit& e);
    void consolidate();

    void grow_vertices(uint64_t vid);
    void grow_faces(uint64_t fid);
    void link(uint64_t v, uint64_t f);
    void unlink(uint64_t v, uint64_t f);
    void rename_in_face(uint64_t f, uint64_t from, uint64_t to);

    std::vector<Eigen::Vector3d> m_vertices;
    std::vector<uint8_t> m_vertex_alive;
    std::vector<std::array<uint64_t, 3>> m_faces;
    std::vector<uint8_t> m_face_alive;
    /// Live faces at each vertex, sorted.
    std::vector<std::vector<uint64_t>> m_vertex_faces;
    /// Each vertex's neighbours across feature edges.
    std::vector<std::vector<uint64_t>> m_feature_nbrs;
    void add_feature(uint64_t a, uint64_t b);
    bool remove_feature(uint64_t a, uint64_t b);
};

} // namespace wmtk::history
