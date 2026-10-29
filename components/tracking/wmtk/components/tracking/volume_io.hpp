#pragma once

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace wmtk::components::tracking {

/// A tet mesh read from a gmsh file (io::read_mesh): each tet once, in the order the file first
/// lists it -- by element tag, which for a file io::write_mesh wrote is the row it was given --
/// with the names of the physical groups that list it (simwild's: a group per tag the tet has,
/// "ambient" for the untagged ones), as an index into the sets of names there are.
struct TetMeshData
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::array<uint64_t, 4>> tets;
    std::vector<uint32_t> groups;
    std::vector<std::set<std::string>> group_sets;
};
TetMeshData read_tet_msh(const std::string& path);

/// Point location in a tet mesh, by a grid over the tets' boxes: the tet holding a point, exactly
/// (volume::locate_in's rules), or one it is out of by a rounding -- across a side of the grid's
/// cells, or of the mesh's box, too.
class TetLocator
{
public:
    /// Shares the mesh, so a locator copied or moved keeps finding in it.
    explicit TetLocator(std::shared_ptr<const TetMeshData> m);
    const TetMeshData& mesh() const { return *m_mesh; }
    /// The tet and the point's barycentric coordinates in it; tet -1 if no tet is near, or p is
    /// not a number.
    std::pair<int64_t, Eigen::Vector4d> locate(const Eigen::Vector3d& p) const;
    /// locate(), preferring a tet in group set `group` among those holding p to a rounding (a
    /// point on a face between two regions); any tet if none of the group does.
    std::pair<int64_t, Eigen::Vector4d> locate(const Eigen::Vector3d& p, uint32_t group) const;
    /// Whether a tet in group set `group` holds p, its faces included, to a rounding: a point
    /// on a face between two regions is in both.
    bool in_group(const Eigen::Vector3d& p, uint32_t group) const;

private:
    std::shared_ptr<const TetMeshData> m_mesh;
    Eigen::Vector3d m_lo = Eigen::Vector3d::Zero();
    double m_h = 1;
    /// A rounding of the largest tet: the farthest a point is from a tet it is a rounding off.
    double m_eps = 0;
    std::array<int64_t, 3> m_n = {{1, 1, 1}};
    std::vector<std::vector<uint32_t>> m_cells;
};

} // namespace wmtk::components::tracking
