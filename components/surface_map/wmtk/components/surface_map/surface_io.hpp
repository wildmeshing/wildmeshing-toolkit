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
    /// Field name -> per face, one value per corner, corner after corner in the face's order:
    /// gmsh element-node data. Texture coordinates live here, a seam being where the two faces
    /// at an edge disagree; they are stored as (u, v, 0), gmsh's views taking 1, 3 or 9
    /// components.
    std::map<std::string, std::vector<std::vector<double>>> corner_fields;
};

/// The triangles of a gmsh file in file order, and every per-face field stored with them. The
/// file must be a single-block surface, as wmtk writes one (MshData::is_single_surface_block).
SurfaceData read_surface_msh(const std::string& path);

/**
 * @brief The surface the files hold, as they hold it: one single-block gmsh file with its fields
 * (read_surface_msh), or triangle meshes of any format, concatenated, neither welded nor cleaned
 * -- the way an application's input history starts -- with no fields.
 */
SurfaceData read_surface(const std::vector<std::string>& paths);

void write_surface_msh(const std::string& path, const SurfaceData& s);

/**
 * @brief `s` added to a copy of the tet mesh in the gmsh file `tet_mesh`, written to `path`: its
 * triangles as an element block on the tet mesh's own nodes, its fields on them. Every vertex of
 * `s` must be a node of the tet mesh, at the same position -- as the tracked surface of a tet
 * mesh's history is.
 */
void write_surface_into_msh(
    const std::string& tet_mesh,
    const std::string& path,
    const SurfaceData& s);

/**
 * @brief The surface as a Wavefront OBJ, textured: the first two components of the per-corner
 * field `uv_field` are its texture coordinates, and a material file next to it (same name,
 * .mtl) maps `texture` onto it.
 */
void write_surface_obj(
    const std::string& path,
    const SurfaceData& s,
    const std::string& uv_field,
    const std::string& texture);

} // namespace wmtk::components::surface_map
