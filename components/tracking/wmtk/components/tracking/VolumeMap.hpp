#pragma once

#include "SurfaceMap.hpp"

#include <wmtk/history/VolumeHistory.hpp>

#include <Eigen/Core>

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace wmtk::components::tracking {

/// A point of a tet mesh: where it is, and its region, an index into the volume history's
/// regions (VolumeHistory::regions, each a set of tags) -- or kAnyRegion: the region of the tet
/// holding it, where an operation's tets first do (a point on a surface between two regions
/// then goes either way). A position that is not a number (the default) is no point: a map
/// leaves it as it is.
struct VolumePoint
{
    static constexpr uint32_t kAnyRegion = uint32_t(-1);
    Eigen::Vector3d position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    uint32_t region = kAnyRegion;
};

/// What building and using a volume map found.
struct VolumeMapStats
{
    /// Operations whose map was built (each is built when points first need it, and kept for
    /// the next while VolumeMapOptions::cache_bytes has room).
    size_t built = 0;
    /// Operations whose map could not be built: their points keep their place, which is not a
    /// bijection of the region there. With the reason, by count.
    size_t failed = 0;
    std::map<std::string, size_t> failures;
    /// Patch pairs mapped on their real geometry, and on a fallback embedding.
    size_t real_pairs = 0, fallback_pairs = 0;
    /// Collapses mapped through the decomposition that splits chords.
    size_t decompositions = 0;
    /// Operations whose map is a bijection of the volume that does not keep the tracked surface:
    /// a face the surface map's cells map lies inside a patch, not on its boundary -- at the
    /// boundary of an open sheet, where both sides of the surface are one side of the patch, or
    /// where no pair keeping the surface could be built (a weld), and one pair over the whole
    /// operation maps it.
    size_t surface_not_kept = 0;
    /// Points that ended in a tet of another region, which a bijection of each region never
    /// does -- through a map that does not keep the surface, the point is in that region from
    /// then on; points whose op map failed.
    size_t region_changed = 0, through_failed = 0;
    /// The operations' maps kept for later calls (VolumeMapOptions::cache_bytes), and about the
    /// bytes they take, now.
    size_t kept = 0, kept_bytes = 0;
};

/// How a VolumeMap keeps what it builds.
struct VolumeMapOptions
{
    /// The operations' maps are built when points first go through them, and kept, for the
    /// points of later calls, up to about this many bytes in all; past it, those used longest
    /// ago are dropped, to be built again if needed. 0 keeps none past the call that built it.
    size_t cache_bytes = size_t(256) << 20;
};

/**
 * @brief The map between the first and the last state of a tet mesh's volume, region by region,
 * rebuilt from its surface history and its volume history (history::VolumeHistory).
 *
 * An operation that does not change the tracked surface leaves every region where it is: the
 * identity in space. One that does gets a local map on the tets around it, built per region
 * from pairs of patches (before -> after): each patch embedded star-shaped -- its real geometry
 * when it is, else a fallback embedding (see volume::spherical) -- and points carried along rays
 * from the centres by the boundary map, the identity on the faces the operation keeps and the
 * surface map's local map (SurfaceMap, CollapsePatch::removed_star) on the tracked faces it
 * changes. Both sides of the surface use the same surface map, so they agree on it. A collapse
 * whose side has a chord goes through a decomposition (split, collapse, unsplit), a swap that
 * cannot be one pair through an exact split and a collapse.
 *
 * The const queries -- to_output(), to_input(), build_all(), op_kinds(), stats() -- may be
 * called on one map from several threads at once: what they share, the operations built and
 * kept and the stats, is kept under a mutex, and each call counts its points' stats apart and
 * adds them at its end.
 */
class VolumeMap
{
public:
    VolumeMap(
        const history::SurfaceHistory& surface,
        history::VolumeHistory volume,
        const VolumeMapOptions& options = {});
    ~VolumeMap();
    VolumeMap(VolumeMap&&) noexcept;

    /// Points of the first state, mapped onto the last.
    std::vector<VolumePoint> to_output(std::vector<VolumePoint> points) const;
    /// Points of the last state, mapped onto the first.
    std::vector<VolumePoint> to_input(std::vector<VolumePoint> points) const;

    const history::VolumeHistory& history() const { return m_volume; }
    /// The region of a tag set, if the history has it.
    std::optional<uint32_t> region_of(std::vector<int64_t> tags) const;
    /// The region of a tet a mesh file lists in the groups `names` -- the tags' labels
    /// (VolumeHistory::tag_names), as io::collect_tags gives them; none, or "ambient" when no tag
    /// has that label, for a tet with no tag -- if the history has it.
    std::optional<uint32_t> region_of_groups(const std::set<std::string>& names) const;
    const SurfaceMap& surface_map() const { return m_surface; }
    /// What building and using the map found so far.
    VolumeMapStats stats() const;
    /// Builds every operation's map, for the stats.
    void build_all() const;
    /// The kinds of the patch pairs operation k's map is made of ("single", "move", "swap
    /// side", a decomposition's steps, ...); none if it cannot be built. For tests and reports.
    std::vector<std::string> op_kinds(size_t k) const;

    /// One operation's map, and its tracked faces before and after (in VolumeMap.cpp).
    struct OpMap;
    struct Faces;

private:
    std::vector<VolumePoint> map(std::vector<VolumePoint> points, bool forward) const;
    /// Operation k's map, from the cache or built; nothing if it cannot be built. The stats
    /// count each operation once, however often it is built.
    std::shared_ptr<const OpMap> op(size_t k) const;

    history::VolumeHistory m_volume;
    VolumeMapOptions m_options;
    SurfaceMap m_surface;
    /// Per volume edit: its tracked faces before and after, and the box of its positions,
    /// before and after it, which a point must be in to be in its tets.
    std::vector<std::unique_ptr<Faces>> m_faces;
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> m_boxes;
    /// The grid a call finds the points in an operation's box with: over the boxes of all the
    /// operations -- a point outside them is in none -- its cells as wide as the median box.
    Eigen::Vector3d m_grid_lo = Eigen::Vector3d::Zero(), m_grid_hi = Eigen::Vector3d::Zero();
    double m_grid_cell = 1;
    /// What the const queries share (VolumeMap.cpp): which operations were built and which
    /// failed, the maps kept, and the stats, with the mutex they are kept under.
    struct Shared;
    std::unique_ptr<Shared> m_shared;
};

} // namespace wmtk::components::tracking
