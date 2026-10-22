#pragma once

#include "SurfaceMap.hpp"

#include <wmtk/history/History.hpp>

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

namespace wmtk::components::tracking {

/// A point on a curve network: a segment id, and barycentric coordinates with respect to the
/// segment's corners in the order the segment stores them.
struct CurvePoint
{
    uint64_t segment;
    Eigen::Vector2d bary;
};

/// Where a curve map stops being a bijection. Counted, never fatal: the map still answers.
struct CurveMapWarnings
{
    /// Operations that changed the curves' topology -- welded two curves, merged or opened a
    /// junction, closed a loop -- whose segments are mapped by relabelling alone, where they
    /// still exist.
    size_t topology = 0;
    /// Segments that became one with another -- duplicates a weld or a collapse leaves: both map
    /// onto it, and it maps back onto the first.
    size_t merged_segments = 0;
    /// Segments an embed laid on more than one segment before it, where input segments overlap:
    /// they map back onto the first.
    size_t overlapping = 0;
    /// A mapped point on a segment an operation removed with nothing to map it to, or that fell
    /// outside every segment it could go to.
    size_t unlocated = 0;
    /// Segments a change no operation describes removed or added
    /// (history::EditKind::replace_faces): points there have nowhere to go.
    size_t replaced = 0;

    size_t total() const { return topology + merged_segments + overlapping + unlocated + replaced; }
};

/// How a CurveMap is built.
struct CurveMapOptions
{
    /// How the map follows a vertex smoothing step (EditKind::move_vertices): barycentric keeps
    /// each point at its segment and barycentric coordinates, riding along with the vertex;
    /// flatten keeps it where it was along the curve, by arc length between the vertices that
    /// stay, as every other operation is mapped.
    Smoothing smoothing = Smoothing::flatten;
};

/**
 * @brief The map between the first and the last state of a CurveHistory.
 *
 * Built from the history alone, one local map per operation: BijectiveRemesh in one dimension,
 * where a joint flattening is exact. An operation changes the segments around a few vertices;
 * between the vertices it leaves in place -- untouched ones, and junctions, ends and feature
 * points, which split the curves into chains -- the curve before and the curve after are two
 * polylines between the same two vertices, and the local map takes one onto the other by
 * normalised arc length: a bijection by construction, with constant stretch along the chain. A
 * split, a collapse and a smoothing step are all mapped so. An operation that changes the
 * topology leaves chains with no counterpart; their segments are mapped by relabelling where
 * they still exist, and counted (CurveMapWarnings::topology). A relabel and a consolidation
 * renumber segments; an embed maps each point to the same place on the segment it lies on.
 */
class CurveMap
{
public:
    explicit CurveMap(const history::CurveHistory& h, const CurveMapOptions& options = {});

    /// Points on the last state, mapped onto the first.
    std::vector<CurvePoint> to_input(std::vector<CurvePoint> points) const;
    /// Points on the first state, mapped onto the last.
    std::vector<CurvePoint> to_output(std::vector<CurvePoint> points) const;

    /// What building the map found.
    const CurveMapWarnings& build_warnings() const { return m_build_warnings; }
    /// The edits (indices into the history) whose local map raised a build warning.
    const std::vector<size_t>& warned_edits() const { return m_warned_edits; }
    /// What the last to_input() / to_output() found.
    const CurveMapWarnings& map_warnings() const { return m_map_warnings; }

    /// The curves at the start and at the end, every slot, as the history holds them.
    const std::vector<Eigen::Vector3d>& input_vertices() const { return m_in_vertices; }
    const std::vector<std::array<uint64_t, 2>>& input_segments() const { return m_in_segments; }
    const std::vector<uint8_t>& input_segment_alive() const { return m_in_alive; }
    const std::vector<Eigen::Vector3d>& output_vertices() const { return m_out_vertices; }
    const std::vector<std::array<uint64_t, 2>>& output_segments() const { return m_out_segments; }
    const std::vector<uint8_t>& output_segment_alive() const { return m_out_alive; }

    /// Each feature point of the first state (History::features) and the vertex of the last
    /// state it became -- its vertex followed through collapses, relabels and embeds -- or
    /// history::kNoVertex if it is gone.
    const std::vector<std::array<uint64_t, 2>>& feature_points() const { return m_features; }
    /// The feature points of the last state.
    const std::vector<uint64_t>& output_feature_points() const { return m_out_features; }

    /// How many operations needed a local map, by kind (index: history::EditKind).
    const std::array<size_t, 16>& steps_by_kind() const { return m_steps_by_kind; }

private:
    /// A polyline: segments end to end, each run from its first corner to its second or back.
    struct Chain
    {
        std::vector<uint64_t> segments;
        std::vector<uint8_t> forward;
        /// The arc length at each segment's start, and each segment's length, as measure() counts
        /// them.
        std::vector<double> start, length;
        double total = 0;
    };
    /// Sets a chain's starts and total from its segments' lengths, each counting for at least a
    /// billionth of the chain's mean segment -- or all alike, if the chain has no length. A
    /// segment of no length in doubles, two vertices the arrangement set apart by less than
    /// their rounding, then keeps a sliver of the chain, and the map stays one to one; any other
    /// chain is measured by its arc length exactly.
    static void measure(Chain& c);
    /// A segment's image: another segment, its corners the same way round or not; kNone for
    /// none.
    struct Image
    {
        uint64_t segment;
        bool flipped;
    };
    /// Where a segment is on a step's chains: the pair, and its place on the chain.
    using Place = std::pair<uint32_t, uint32_t>;
    struct Step
    {
        history::EditKind kind;
        /// Local maps: chains before and after, paired, and where each segment is on them,
        /// sorted by segment.
        std::vector<std::pair<Chain, Chain>> chains;
        std::vector<std::pair<uint64_t, Place>> before_at, after_at;
        /// Local maps: segments mapped by relabelling, or to nothing, sorted by segment. Any
        /// other segment maps to itself.
        std::vector<std::pair<uint64_t, Image>> forward, backward;
        /// consolidate, relabel: every segment's new id, and back; any other has none.
        std::vector<Image> old_to_new, new_to_old;
        /// embed: the segments before and after, by their corners' positions, and which of the
        /// other side's each lies on -- for those whose segments after lie end to end, the
        /// chains above instead: each segment before paired with them.
        std::vector<std::array<Eigen::Vector3d, 2>> before, after;
        std::vector<std::vector<uint64_t>> to_after, to_before;
    };

    void build_local(history::CurveReplay& r, const history::Edit& e, Step& step, bool flatten);
    void build_renumbering(history::CurveReplay& r, const history::Edit& e, Step& step);
    void build_embed(history::CurveReplay& r, const history::Edit& e, Step& step);
    void build_replace(history::CurveReplay& r, const history::Edit& e, Step& step);

    /// One point through one step, or back; false if it has no image.
    static bool forward(const Step& s, CurvePoint& p);
    static bool backward(const Step& s, CurvePoint& p);
    /// Points through every step, forward or back, counting those left without an image. A
    /// local step moves only the points on its own segments, which an index finds.
    void map(std::vector<CurvePoint>& points, bool to_output) const;
    /// The point at the same fraction of arc length along `to` as `bary` on segment i of `from`.
    static CurvePoint
    along(const Chain& from, size_t i, const Eigen::Vector2d& bary, const Chain& to);

    std::vector<Step> m_steps;
    CurveMapWarnings m_build_warnings;
    std::vector<size_t> m_warned_edits;
    mutable CurveMapWarnings m_map_warnings;
    std::vector<Eigen::Vector3d> m_in_vertices, m_out_vertices;
    std::vector<std::array<uint64_t, 2>> m_in_segments, m_out_segments;
    std::vector<uint8_t> m_in_alive, m_out_alive;
    std::vector<std::array<uint64_t, 2>> m_features;
    std::vector<uint64_t> m_out_features;
    std::array<size_t, 16> m_steps_by_kind = {};
};

} // namespace wmtk::components::tracking
