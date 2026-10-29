#pragma once

#include "History.hpp"

#include <array>
#include <cstdint>
#include <set>
#include <vector>

namespace wmtk::history {

/**
 * @brief A complex driven by the edits of a History: a surface (N = 3) or a curve network
 * (N = 2).
 *
 * Starts from the history's snapshot and applies edits one at a time, with the semantics the
 * meshes give their operations (see EditKind), so after the k-th edit it holds exactly the
 * recording mesh's state after its k-th committed operation: the same vertex and face ids, the
 * same corner order in every face, the same positions.
 */
template <int N>
class BasicReplay
{
public:
    using Face = typename History<N>::Face;
    using Feature = typename History<N>::Feature;

    explicit BasicReplay(const History<N>& h);

    void apply(const Edit& e);
    /// Applies every edit of `h`, which must be the history this was built from.
    void apply_all(const History<N>& h);

    const std::vector<Eigen::Vector3d>& vertices() const { return m_vertices; }
    const std::vector<uint8_t>& vertex_alive() const { return m_vertex_alive; }
    const std::vector<Face>& faces() const { return m_faces; }
    const std::vector<uint8_t>& face_alive() const { return m_face_alive; }
    /// The live faces at vertex `v`, sorted.
    const std::vector<uint64_t>& faces_of(uint64_t v) const { return m_vertex_faces[v]; }
    /// The features (see History::features), each with its vertices sorted.
    std::set<Feature> features() const;
    bool is_feature(Feature f) const;

private:
    void split_edge(const Edit& e);
    void split_face(const Edit& e);
    void collapse_edge(const Edit& e);
    void swap_edge(const Edit& e);
    void replace_faces(const Edit& e);
    void relabel(const Edit& e);
    void embed(const Edit& e);
    void consolidate();

    void grow_vertices(uint64_t vid);
    void grow_faces(uint64_t fid);
    void link(uint64_t v, uint64_t f);
    void unlink(uint64_t v, uint64_t f);
    void rename_in_face(uint64_t f, uint64_t from, uint64_t to);
    /// Retires each of `vs` that has no face left -- and, on a curve, carries no feature point --
    /// with its features.
    void retire_bare(const std::vector<uint64_t>& vs);

    std::vector<Eigen::Vector3d> m_vertices;
    std::vector<uint8_t> m_vertex_alive;
    std::vector<Face> m_faces;
    std::vector<uint8_t> m_face_alive;
    /// Live faces at each vertex, sorted.
    std::vector<std::vector<uint64_t>> m_vertex_faces;
    /// The features at each vertex: on a surface, its neighbours across feature edges; on a
    /// curve, itself if it is a feature point.
    std::vector<std::vector<uint64_t>> m_feature_nbrs;
    void add_feature(const Feature& f);
    bool remove_feature(const Feature& f);
    /// Every feature at `from` now at `to` instead; an edge left with one vertex goes.
    void rename_features(uint64_t from, uint64_t to);
    /// Removes every feature at `v`.
    void drop_features(uint64_t v);
};

} // namespace wmtk::history
