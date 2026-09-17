#include "prismatic_mesh.hpp"

#include <paraviewo/VTUWriter.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {

void write_prismatic_mesh(const PrismaticMeshInput& input, const std::filesystem::path& path)
{
    const auto active_vertices = input.mesh->get_vertices();
    const size_t n = active_vertices.size();
    MatrixXd vertices(n, 3);
    VectorXd tags(n), ids(n), correspondence(n);
    std::vector<int> output_index(input.vertices.rows(), -1);
    for (size_t i = 0; i < n; ++i) {
        const size_t v = active_vertices[i].vid(*input.mesh);
        output_index[v] = static_cast<int>(i);
        vertices.row(i) = input.vertices.row(v);
        tags[i] = input.vertex_tags[v];
        ids[i] = static_cast<double>(input.source_vertex_ids[v]);
        correspondence[i] = static_cast<double>(input.corr_input_vid[v]);
    }
    // Compact output row indices only. vid and corr_input_vid remain source IDs.
    MatrixXi tetrahedra = input.tetrahedra;
    for (Eigen::Index i = 0; i < tetrahedra.rows(); ++i) {
        for (int j = 0; j < 4; ++j) {
            tetrahedra(i, j) = output_index.at(input.tetrahedra(i, j));
            if (tetrahedra(i, j) < 0) {
                log_and_throw_error("Output tetrahedron references an inactive vertex.");
            }
        }
    }
    VectorXd input_tags(tetrahedra.rows()), offset_tags(tetrahedra.rows());
    for (Eigen::Index i = 0; i < tetrahedra.rows(); ++i) {
        input_tags[i] = input.input_cells[i];
        offset_tags[i] = input.offset_tet_tags[i];
    }
    paraviewo::VTUWriter writer;
    writer.add_field("labels", tags);
    writer.add_field("vid", ids);
    writer.add_field("corr_input_vid", correspondence);
    writer.add_cell_field("tag_0", input_tags);
    writer.add_cell_field("offset_tag", offset_tags);
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    if (!writer.write_mesh(path.string(), vertices, tetrahedra, paraviewo::CellType::Tetrahedron)) {
        log_and_throw_error("Could not write result mesh: {}", path.string());
    }
}

} // namespace wmtk::components::prismatic_mesh
