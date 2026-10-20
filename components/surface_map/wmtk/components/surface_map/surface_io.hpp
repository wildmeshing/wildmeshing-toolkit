#pragma once

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace wmtk::components::surface_map {

/// A triangle surface and named per-face fields, as read from or written to a gmsh file.
struct SurfaceData
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::array<uint64_t, 3>> faces;
    /// Field name -> one value per face. A value has 1, 3 or 9 components (a scalar such as a
    /// label, a vector, a tensor), the ones gmsh element data can hold.
    std::map<std::string, std::vector<std::vector<double>>> face_fields;
};

/// The triangles of a gmsh file in file order, and every per-face field stored with them.
SurfaceData read_surface_msh(const std::string& path);

void write_surface_msh(const std::string& path, const SurfaceData& s);

} // namespace wmtk::components::surface_map
