#pragma once

#include <wmtk/history/SurfaceHistory.hpp>

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <set>
#include <vector>

namespace wmtk::components::surface_map {

/// A point on a triangle complex: a face id, and barycentric coordinates with respect to the
/// face's corners in the order the face stores them.
struct SurfacePoint
{
    uint64_t face;
    Eigen::Vector3d bary;
};

/// Where a map stops being a bijection. Counted, never fatal: the map still answers.
struct MapWarnings
{
    /// A collapse whose patch is not a disk; its faces are mapped by relabelling alone.
    size_t not_a_disk = 0;
    /// A collapse along the boundary or a curve with no triangle after it holding the edge that
    /// replaces the removed vertex's two: flattened onto a chord instead, where a triangle with
    /// all three corners on it goes flat.
    size_t flat_triangle = 0;
    /// An atlas with a triangle that is not positively oriented.
    size_t inverted = 0;
    /// A collapse that merged duplicate faces: two faces map onto one.
    size_t merged_faces = 0;
    /// A mapped point that fell outside every face of the patch it was mapped into, or onto a
    /// face that an operation removed with nothing to map it to.
    size_t unlocated = 0;
    /// Faces a change no operation describes removed or added (history::EditKind::replace_faces):
    /// points there have nowhere to go.
    size_t replaced = 0;

    size_t total() const
    {
        return not_a_disk + flat_triangle + inverted + merged_faces + unlocated + replaced;
    }
};

/**
 * @brief The map between the first and the last state of a SurfaceHistory.
 *
 * Built from the history alone, one local map per operation (BijectiveRemesh). A vertex move
 * keeps every (face, barycentric) point where it is; a split maps its new faces affinely onto
 * the face they subdivide, with the new vertex where the history put it; a collapse and a swap
 * map through a joint Tutte embedding of the faces they touch, before and after, on a shared
 * convex outline -- a boundary vertex the collapse removes sits on the straight segment between
 * its boundary neighbours, so the boundary maps onto itself. A consolidation renumbers faces.
 */
class SurfaceMap
{
public:
    explicit SurfaceMap(const history::SurfaceHistory& h);

    /// Points on the last state, mapped onto the first.
    std::vector<SurfacePoint> to_input(std::vector<SurfacePoint> points) const;
    /// Points on the first state, mapped onto the last.
    std::vector<SurfacePoint> to_output(std::vector<SurfacePoint> points) const;

    /// What building the map found.
    const MapWarnings& build_warnings() const { return m_build_warnings; }
    /// The edits (indices into the history) whose local map raised a build warning.
    const std::vector<size_t>& warned_edits() const { return m_warned_edits; }
    /// What the last to_input() / to_output() found.
    const MapWarnings& map_warnings() const { return m_map_warnings; }

    /// The complex at the start and at the end, every slot, as the history holds them.
    const std::vector<Eigen::Vector3d>& input_vertices() const { return m_in_vertices; }
    const std::vector<std::array<uint64_t, 3>>& input_faces() const { return m_in_faces; }
    const std::vector<uint8_t>& input_face_alive() const { return m_in_face_alive; }
    const std::vector<Eigen::Vector3d>& output_vertices() const { return m_out_vertices; }
    const std::vector<std::array<uint64_t, 3>>& output_faces() const { return m_out_faces; }
    const std::vector<uint8_t>& output_face_alive() const { return m_out_face_alive; }
    /// The feature edges at the end, smaller vertex id first.
    const std::set<std::array<uint64_t, 2>>& output_feature_edges() const { return m_out_features; }

    /// How many operations needed a local map, by kind (index: history::EditKind).
    const std::array<size_t, 8>& cells_by_kind() const { return m_cells_by_kind; }
    /// Sides of collapses that removed a vertex on the boundary or on a feature curve. Its two
    /// edges along the curve become one; the map splits the triangle holding that new edge
    /// virtually where the removed vertex was, so both sides keep their area.
    size_t curve_collapses() const { return m_curve_collapses; }
    /// Vertex moves whose points keep their barycentric coordinates instead of their place: a
    /// vertex on more than one curve, or whose star is not a disk.
    size_t kept_moves() const { return m_kept_moves; }
    /// Local maps between patches one of which is folded over in 3D: a bijection still, but
    /// stretched, since no flat layout matches a fold.
    size_t folded_cells() const { return m_folded_cells; }

private:
    /// A face of a patch with the domain position of each of its corners -- or a piece of one:
    /// a face the map splits virtually appears as several, each with its corners given as
    /// barycentric coordinates in the face.
    struct Face
    {
        uint64_t fid;
        std::array<Eigen::Vector2d, 3> uv;
        std::array<Eigen::Vector3d, 3> sub = {
            {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()}};
    };
    /// One flattened patch: the faces it covers before and after an edit, in a shared domain.
    struct Cell
    {
        std::vector<Face> before, after;
    };
    struct Step
    {
        history::EditKind kind;
        std::vector<Cell> cells;
        /// consolidate only: face renumbering.
        std::vector<uint64_t> old_to_new, new_to_old;
        /// Faces this step removes with nothing to map their points to (a collapse that could
        /// only be mapped by relabelling, a replace_faces).
        std::vector<uint64_t> dead;
        /// Faces this step adds from nothing (a replace_faces): mapped back, their points have
        /// nowhere to go.
        std::vector<uint64_t> born;
    };

    std::vector<SurfacePoint> map(std::vector<SurfacePoint> points, bool forward) const;

    std::vector<Step> m_steps;
    MapWarnings m_build_warnings;
    std::vector<size_t> m_warned_edits;
    mutable MapWarnings m_map_warnings;
    std::array<size_t, 8> m_cells_by_kind = {};
    size_t m_curve_collapses = 0;
    size_t m_kept_moves = 0;
    size_t m_folded_cells = 0;

    std::vector<Eigen::Vector3d> m_in_vertices, m_out_vertices;
    std::vector<std::array<uint64_t, 3>> m_in_faces, m_out_faces;
    std::vector<uint8_t> m_in_face_alive, m_out_face_alive;
    std::set<std::array<uint64_t, 2>> m_out_features;
};

} // namespace wmtk::components::surface_map
