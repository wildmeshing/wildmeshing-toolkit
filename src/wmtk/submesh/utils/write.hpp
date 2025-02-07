#pragma once

#include <filesystem>

namespace wmtk::submesh {
class SubMesh;
class Embedding;
} // namespace wmtk::submesh


namespace wmtk::submesh::utils {

void write(
    const std::filesystem::path& filename,
    const std::string& vertices_name,
    const SubMesh& sub,
    bool write_points = true,
    bool write_edges = true,
    bool write_faces = true,
    bool write_tetrahedra = true);

void write(
    const std::filesystem::path& filename,
    const std::string& vertices_name,
    const Embedding& emb,
    bool write_points = true,
    bool write_edges = true,
    bool write_faces = true,
    bool write_tetrahedra = true);

} // namespace wmtk::submesh::utils
