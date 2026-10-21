#pragma once

#include "Replay.hpp"
#include "SurfaceHistory.hpp"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <vector>

namespace wmtk::history {

/**
 * @brief Records a tracked surface: a triangle complex made of some of the faces of a mesh that
 * gives them no ids of its own -- the tracked faces of a tet mesh.
 *
 * The surface gets ids here: one for each mesh vertex on it, one for each face, so the history
 * is in the terms a TriMesh records in (SurfaceHistory, replayed by Replay) and a renumbering of
 * the mesh changes nothing in it. At each commit an operation reports the tracked faces around
 * it before and after, by mesh vertex ids. The recorder finds the edit that takes the one to the
 * other -- an edge split, an edge collapse, an edge swap -- checks that it gives exactly the
 * faces after, and records it; a change no edit fits it records as it is
 * (EditKind::replace_faces), which a map cannot follow. A face keeps the corner order it had,
 * through the edits' rules; one that only a replace_faces makes takes the order `orient` gives.
 *
 * Commits may come concurrently from operations whose regions do not overlap. They are
 * serialised here, while each operation still holds its locks, so the order of the edits is a
 * valid serial order of the run, as for Recorder.
 */
class TrackedSurfaceRecorder
{
public:
    /// A face by mesh vertex ids.
    using Triangle = std::array<size_t, 3>;
    using Positions = std::function<Eigen::Vector3d(size_t)>;
    /// A tracked face's corners, in the order that orients it.
    using Orient = std::function<Triangle(const Triangle&)>;

    /// The tracked surface now, any corner order; `orient` orders the faces of the snapshot.
    TrackedSurfaceRecorder(const std::vector<Triangle>& faces, Positions position, Orient orient);

    /// What an operation did to the tracked surface, as the mesh saw it at its commit point.
    struct Change
    {
        /// split_edge, collapse_edge, swap_edge (for every kind of swap) or move_vertices.
        EditKind kind = EditKind::move_vertices;
        /// split_edge: {v0, v1, new}; collapse_edge: {removed, kept}; otherwise empty.
        std::vector<size_t> v;
        /// The tracked faces around it before and after, any corner order. A face in both is
        /// left as it is.
        std::vector<Triangle> before, after;
        /// The mesh vertices it wrote. Those on the surface afterwards are recorded where they
        /// are now, if they moved.
        std::vector<size_t> written;
    };
    void commit(Change&& c);

    /// The mesh renumbered its vertices: old_to_new[v], size_t(-1) for a vertex gone.
    void renumber(const std::vector<size_t>& old_to_new);

    /// Brings the surface in line with `faces`, every tracked face of the mesh, after a change
    /// outside the operations: one replace_faces edit, if anything differs. Returns how many
    /// faces it removed and added.
    size_t resync(const std::vector<Triangle>& faces);

    /// How the commits were recorded.
    struct Stats
    {
        size_t split = 0, collapse = 0, swap = 0, move = 0;
        /// A collapse that welded two surface vertices with no tracked face on its edge.
        size_t weld = 0;
        /// Changes no edit fitted, recorded as they are.
        size_t replaced = 0;
        /// Faces the mesh reported gone that the recorder did not know: it had lost track.
        size_t unknown = 0;
    };
    Stats stats() const;

    /// Not synchronised: read once the mesh is done.
    const SurfaceHistory& history() const { return m_history; }
    SurfaceHistory& history() { return m_history; }
    /// The surface as it is now.
    const Replay& surface() const { return m_replay; }

    /// The surface face a mesh face is, if it is one the recorder knows. Not synchronised.
    std::optional<uint64_t> face_id(const Triangle& t) const;
    /// The surface vertex of a mesh vertex, if it is on the surface. Not synchronised.
    std::optional<uint64_t> vertex_id(size_t v) const;

private:
    static constexpr uint64_t kNone = uint64_t(-1);

    std::optional<uint64_t> live(size_t v) const;
    std::optional<uint64_t> lookup(const Triangle& t) const;
    uint64_t fresh_vertex(size_t v);
    std::optional<Edit> as_split(
        const Change& c,
        const std::vector<Triangle>& before,
        const std::vector<Triangle>& removed,
        const std::vector<Triangle>& added,
        std::vector<uint64_t>& fresh);
    std::optional<Edit>
    as_collapse(Change& c, const std::vector<Triangle>& before, const std::vector<Triangle>& after);
    std::optional<Edit> as_swap(
        const std::vector<Triangle>& removed,
        const std::vector<Triangle>& added);
    /// Records `e` -- applies it, then the moves of `written` and of `fresh` vertices.
    void record(Edit&& e, const std::vector<size_t>& written, const std::vector<uint64_t>& fresh);
    /// The replace_faces edit taking away `removed` and adding `added` (mesh ids, sorted).
    Edit replacement(
        const std::vector<Triangle>& removed,
        const std::vector<Triangle>& added,
        std::vector<uint64_t>& fresh);

    mutable std::mutex m_mutex;
    Positions m_position;
    Orient m_orient;
    /// Mesh vertex -> surface vertex, and back. Before the history: the snapshot fills them.
    std::vector<uint64_t> m_surface_of;
    std::vector<size_t> m_mesh_of;
    SurfaceHistory m_history;
    Replay m_replay;
    /// The last position recorded for each surface vertex.
    std::vector<Eigen::Vector3d> m_known;
    uint64_t m_next_face = 0;
    Stats m_stats;
};

} // namespace wmtk::history
