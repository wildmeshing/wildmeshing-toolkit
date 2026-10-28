#pragma once

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace wmtk::components::tracking {

/// A tet mesh read from a gmsh file: each tet once, with the names of the physical groups that
/// list it (simwild lists a tet once per tag it has, and the untagged ones as "ambient"), as an
/// index into the sets of names there are.
struct TetMeshData
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::array<uint64_t, 4>> tets;
    std::vector<uint32_t> groups;
    std::vector<std::set<std::string>> group_sets;
};
TetMeshData read_tet_msh(const std::string& path);

/// Point location in a tet mesh: the tet holding a point (the one whose smallest barycentric
/// coordinate is largest among those near it), by a grid over the tets' boxes.
class TetLocator
{
public:
    explicit TetLocator(const TetMeshData& m);
    /// The tet and the point's barycentric coordinates in it; tet -1 if no tet is near.
    std::pair<int64_t, Eigen::Vector4d> locate(const Eigen::Vector3d& p) const;
    /// Whether a tet in group set `group` holds p, its faces included, to rounding (barycentric
    /// coordinates down to -1e-9): a point on a face between two regions is in both.
    bool in_group(const Eigen::Vector3d& p, uint32_t group) const;

private:
    const TetMeshData& m_mesh;
    Eigen::Vector3d m_lo;
    double m_h = 1;
    std::array<int64_t, 3> m_n = {{1, 1, 1}};
    std::vector<std::vector<uint32_t>> m_cells;
};

} // namespace wmtk::components::tracking
