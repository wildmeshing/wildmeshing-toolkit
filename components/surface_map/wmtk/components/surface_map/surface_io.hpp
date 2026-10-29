#pragma once

#include <wmtk/MeshData.hpp>

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace wmtk::components::surface_map {

/// A triangle surface and named per-face fields, as read from or written to a mesh.
struct SurfaceData
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::array<uint64_t, 3>> faces;
    /// Field name -> one value per face, of as many components as the field has columns: a
    /// label, a vector, a tensor.
    std::map<std::string, std::vector<std::vector<double>>> face_fields;
    /// Field name -> per face, one value per corner, corner after corner in the face's order
    /// (is_corner_field): texture coordinates, say, a seam being where the two faces at an edge
    /// disagree. In a mesh, a triangle field of 3k columns.
    std::map<std::string, std::vector<std::vector<double>>> corner_fields;

    /// How a field is stored in a mesh: its type (an index of AttributeValues -- 0 Float64,
    /// 1 Int64, 2 UInt8) and the names of its columns. A field not listed is Float64 and
    /// unlabelled.
    struct FieldType
    {
        size_t type = 0;
        std::vector<std::string> labels;
    };
    std::map<std::string, FieldType> field_types;
};

/// Whether a triangle field of a mesh holds a value per corner, corner after corner, rather than
/// one per face: the fields io::read_mesh reads per corner, `uv` (six columns) and `normal`
/// (nine).
bool is_corner_field(const std::string& name);

/**
 * @brief The triangles of `mesh`, in row order, with every field on them: a per-corner one
 * (is_corner_field) as a corner field, every other as a face field. `what` names the mesh in
 * messages. Vertex fields are not carried, other than `vid` (io::read_mesh's source ids); with
 * `strict`, one is refused, as the memory route refuses an attribute it does not use, and
 * otherwise only warned about, as a file reader ignores what it does not read.
 */
SurfaceData surface_from_mesh(const MeshData& mesh, const std::string& what, bool strict);

/// `s` as a mesh: its triangles, its fields on them as triangle attributes of the types
/// field_types gives.
MeshData mesh_from_surface(const SurfaceData& s);

/// The triangles of a gmsh file in row order, and every field stored with them, as io::read_mesh
/// reads them (surface_from_mesh).
SurfaceData read_surface_msh(const std::string& path);

/**
 * @brief The surface the files hold, as they hold it: one gmsh file with its fields
 * (read_surface_msh), or triangle meshes of any format, concatenated, neither welded nor cleaned
 * -- the way an application's input history starts -- with no fields.
 */
SurfaceData read_surface(const std::vector<std::string>& paths);

/// `s` written through io::write_mesh (mesh_from_surface): a gmsh file wm.read_mesh reads.
void write_surface_msh(const std::string& path, const SurfaceData& s);

/**
 * @brief `s` added to a copy of the tet mesh in the gmsh file `tet_mesh`, written to `path`: its
 * triangles as rows of the tet mesh's triangle block, on its own vertices, its fields on them.
 * Every vertex of `s` must be a vertex of the tet mesh, at the same position -- as the tracked
 * surface of a tet mesh's history is.
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
