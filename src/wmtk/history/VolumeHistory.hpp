#pragma once

#include "History.hpp"

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace wmtk::history {

/// The tet operations a volume history records.
enum class VolumeOp : uint8_t {
    collapse = 0,
    swap_32 = 1,
    swap_44 = 2,
    swap_56 = 3,
    swap_23 = 4,
    smooth = 5,
};

/**
 * @brief One operation that changed the tracked surface of a tet mesh, as the tets around it saw
 * it: the tets before and after, their regions, and where their vertices were.
 *
 * Vertices are local to the edit, numbered from 0: a volume map follows points in space, so no
 * vertex identity is needed from one edit to the next. The edit links to the surface history by
 * the range of its edits the operation made, and by the surface vertex of each local vertex, so
 * the tracked faces among the tets, and the surface map's local map of the operation, can be
 * found there.
 */
struct VolumeEdit
{
    VolumeOp op = VolumeOp::collapse;
    /// The surface history's edits the operation made: [first_edit, first_edit + n_edits).
    uint64_t first_edit = 0;
    uint64_t n_edits = 0;
    /// Per local vertex: its position before the operation, and its vertex on the tracked
    /// surface before and after it (kNoVertex: off the surface).
    std::vector<Eigen::Vector3d> positions;
    std::vector<uint64_t> surface_before, surface_after;
    /// The vertices the operation moved, and where to.
    std::vector<std::pair<uint32_t, Eigen::Vector3d>> moved;
    /// The tets around the operation before it, positively oriented, and the region of each
    /// (an index into VolumeHistory::regions).
    std::vector<std::array<uint32_t, 4>> tets;
    std::vector<uint32_t> regions;
    /// After it: those tets but `removed` (indices into `tets`), and `added`.
    std::vector<uint32_t> removed;
    std::vector<std::array<uint32_t, 4>> added;
    std::vector<uint32_t> added_regions;
    /// collapse: {removed, kept}; smooth: {the vertex}; swaps: none.
    std::vector<uint32_t> op_vertices;

    /// The tets after the operation, with their regions.
    std::vector<std::array<uint32_t, 4>> tets_after(
        std::vector<uint32_t>* regions_after = nullptr) const;
    /// The positions after the operation.
    std::vector<Eigen::Vector3d> positions_after() const;
};

/**
 * @brief The tets around every operation that changed a tet mesh's tracked surface, which,
 * with the surface history, is what a map of the volume needs: every other operation leaves
 * each region where it is in space.
 */
struct VolumeHistory
{
    /// The regions, each the set of tags of its tets, sorted.
    std::vector<std::vector<int64_t>> regions;
    /// In the order of the surface edits they made.
    std::vector<VolumeEdit> edits;
    /// The surface history file these edits index into, and the offset of the recorder's first
    /// edit in it (the input's own edits come first there).
    uint64_t surface_edit_offset = 0;

    void save(const std::string& path) const;
    static VolumeHistory load(const std::string& path);
};

/**
 * @brief Records a VolumeHistory alongside a TrackedSurfaceRecorder. An operation captures the
 * tets around it before it changes the mesh and at its commit point (TetMesh does both), and
 * commits them here with the range of surface edits its surface commit made; one that made none
 * is not recorded. Commits may come concurrently; they are ordered by surface edit when saved.
 */
class VolumeRecorder
{
public:
    /// The tets around an operation, by mesh vertex ids, with their tags; each vertex's
    /// position and surface vertex (kNoVertex: none).
    struct Capture
    {
        std::vector<std::array<size_t, 4>> tets;
        std::vector<std::vector<int64_t>> tags;
        std::vector<size_t> vertices;
        std::vector<Eigen::Vector3d> positions;
        std::vector<uint64_t> surface;
    };
    void commit(
        VolumeOp op,
        std::array<size_t, 2> surface_edits,
        const std::vector<size_t>& op_vertices,
        const Capture& before,
        const Capture& after);

    /// Not synchronised: read once the mesh is done. Sorted by surface edit.
    VolumeHistory history() const;
    /// Bytes recorded so far, roughly: what the saved file will take.
    size_t size_estimate() const;

private:
    uint32_t region_of(const std::vector<int64_t>& tags);

    mutable std::mutex m_mutex;
    VolumeHistory m_history;
};

} // namespace wmtk::history
