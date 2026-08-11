#pragma once

#include <atomic>
#include <cstddef>
#include <vector>

namespace wmtk {

/**
 * @brief The rounded/exact bookkeeping shared by every mesh that keeps both coordinates.
 *
 * All four application meshes store a vertex position twice -- exactly, as rationals, and
 * rounded, as doubles -- and carry the same invariant about which of the two the rest of the
 * code may read. This holds that invariant, and the sweep that restores it, in one place.
 *
 * Dimension-free by construction: nothing here mentions a Tuple or a position type, so the
 * three hooks below are all a mesh has to supply. Unlike the improvement loop -- which the two
 * optimizer bases still duplicate, because 2D and 3D genuinely disagree about the stop
 * condition and the cbrt convention -- this really is the same code twice.
 */
class RationalPositions
{
public:
    virtual ~RationalPositions() = default;

    /**
     * @brief Try to round every un-rounded vertex; returns the number reclaimed.
     *
     * Operations round opportunistically, at the vertex they touch (the new vertex of a split,
     * the merged vertex of a collapse), and none of them reaches a vertex that only becomes
     * roundable later -- smoothing skips "good" regions by default. Without a sweep such a
     * vertex keeps exact coordinates all the way into the output for no geometric reason, and
     * split_edge_after introduces them unconditionally whenever the rounded midpoint would
     * invert, so the sweep is what keeps that from reaching the caller.
     *
     * Skipped outright when m_all_rounded says there is nothing to do.
     */
    size_t round_all_vertices();

    /**
     * @brief Run the sweep, then report whether the mesh is now fully rounded.
     *
     * The termination condition of the operation loop. A mesh that hits the quality target
     * while some vertex still carries exact coordinates is not finished, because the output is
     * what the caller consumes and rational coordinates in it are a defect regardless of how
     * good the elements are.
     *
     * This is also what makes the exact-rational fallback in split_edge_after safe: a split is
     * the only operation that can un-round a vertex -- collapse, the swaps and smoothing never
     * do, and the post-optimization pass is collapse-only -- so the loop only has to outlast
     * the sweep.
     *
     * O(1) once m_all_rounded is set, so it is cheap enough to sit on every early-out.
     */
    bool round_and_check_all_rounded();

protected:
    /// Every live vertex, in the mesh's own iteration order.
    virtual std::vector<size_t> all_vertex_ids() const = 0;
    /// Whether this vertex's double position is currently trusted.
    virtual bool vertex_is_rounded(const size_t vid) const = 0;
    /// Try to replace this vertex's exact position with its rounded one; false if that would
    /// invert an incident cell, in which case nothing is changed.
    virtual bool round_vertex(const size_t vid) = 0;

    /**
     * @brief True when every vertex is known to be rounded.
     *
     * Only trusted when true, and only round_all_vertices() sets it that way. Any code that
     * leaves a vertex un-rounded must clear it, or the sweep will skip the vertex forever.
     * Atomic because operations that clear it run in parallel.
     */
    std::atomic<bool> m_all_rounded = false;
};

} // namespace wmtk
