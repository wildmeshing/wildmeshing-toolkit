#pragma once

#include "SurfaceMap.hpp"

#include <wmtk/history/VolumeHistory.hpp>

#include <Eigen/Core>

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
/// then goes either way).
struct VolumePoint
{
    static constexpr uint32_t kAnyRegion = uint32_t(-1);
    Eigen::Vector3d position;
    uint32_t region = kAnyRegion;
};

/// What building and using a volume map found.
struct VolumeMapStats
{
    /// Operations whose map was built (each is built when points need it, and dropped once
    /// they went through).
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
    /// does; points whose op map failed.
    size_t region_changed = 0, through_failed = 0;
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
 */
class VolumeMap
{
public:
    VolumeMap(const history::SurfaceHistory& surface, history::VolumeHistory volume);
    ~VolumeMap();
    VolumeMap(VolumeMap&&) noexcept;

    /// Points of the first state, mapped onto the last.
    std::vector<VolumePoint> to_output(std::vector<VolumePoint> points) const;
    /// Points of the last state, mapped onto the first.
    std::vector<VolumePoint> to_input(std::vector<VolumePoint> points) const;

    const history::VolumeHistory& history() const { return m_volume; }
    /// The region of a tag set, if the history has it.
    std::optional<uint32_t> region_of(std::vector<int64_t> tags) const;
    /// The region of a tet a mesh file lists in the groups `names` -- the tags' names
    /// (VolumeHistory::tag_names), "ambient" for a tet with none -- if the history has it.
    std::optional<uint32_t> region_of_groups(const std::set<std::string>& names) const;
    const SurfaceMap& surface_map() const { return m_surface; }
    const VolumeMapStats& stats() const { return m_stats; }
    /// Builds every operation's map, for the stats.
    void build_all() const;

    /// One operation's map, and its tracked faces before and after (in VolumeMap.cpp).
    struct OpMap;
    struct Faces;

private:
    std::vector<VolumePoint> map(std::vector<VolumePoint> points, bool forward) const;
    /// Operation k's map; nothing if it cannot be built.
    std::unique_ptr<OpMap> op(size_t k) const;

    history::VolumeHistory m_volume;
    SurfaceMap m_surface;
    /// Per volume edit: its tracked faces before and after.
    std::vector<std::unique_ptr<Faces>> m_faces;
    /// Per operation: 0 not built yet, 1 built, 2 failed.
    mutable std::vector<char> m_built;
    mutable VolumeMapStats m_stats;
};

} // namespace wmtk::components::tracking
