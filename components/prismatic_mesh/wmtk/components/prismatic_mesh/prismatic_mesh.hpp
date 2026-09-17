#pragma once

#include <wmtk/TetMesh.h>
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <vector>

namespace wmtk::components::prismatic_mesh {

// Row indices address the loaded mesh. Source IDs are preserved separately and need not
// equal row indices. These arrays describe the initial mesh, before any topology edits.
struct PrismaticMeshInput
{
    std::unique_ptr<TetMesh> mesh;
    MatrixXd vertices;
    MatrixXi tetrahedra;
    std::vector<int> vertex_tags; // -1: other, 1: input, 2: offset
    std::vector<int64_t> source_vertex_ids;
    std::vector<int64_t> corr_input_vid; // source input ID, or -1
    std::vector<int64_t> corr_input_vertex; // resolved mesh row, or -1
    std::vector<size_t> input_vertices;
    std::vector<size_t> offset_vertices;
    std::vector<std::vector<size_t>> input_to_offset_vertices; // indexed by mesh row
    std::vector<int> input_cells; // tag_0
    std::vector<int> offset_tet_tags; // 1: offset band, -1: other; derived from VTU offset_tag
};

// Single-piece tetrahedral VTU, ASCII or uncompressed inline base64 binary.
PrismaticMeshInput load_prismatic_mesh(const std::filesystem::path& path);
// Export active vertices and tetrahedra, preserving source IDs and correspondence.
void write_prismatic_mesh(const PrismaticMeshInput& input, const std::filesystem::path& path);
// Keep band tetrahedra in their original order and filter cell attributes with them.
// Vertex arrays/IDs stay unchanged; unused vertices are inactive in the rebuilt TetMesh.
void keep_offset_band(PrismaticMeshInput& input);
// Main algorithm pipeline, operating on the loaded mesh in place.
void prism_main(PrismaticMeshInput& input);
void prismatic_mesh(nlohmann::json json_params);

} // namespace wmtk::components::prismatic_mesh
