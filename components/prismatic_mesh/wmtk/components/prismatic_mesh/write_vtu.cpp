#include "prismatic_mesh.hpp"

#include <paraviewo/VTUWriter.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {
namespace {
void add_targets(
    paraviewo::VTUWriter& writer,
    const PrismaticMeshInput& input,
    const std::vector<size_t>& vertex_rows)
{
    if (input.vertex_component_ids.empty()) return;
    VectorXd component_ids(vertex_rows.size()), singular(vertex_rows.size());
    MatrixXd normals(vertex_rows.size(), 3), targets(vertex_rows.size(), 3);
    for (size_t i = 0; i < vertex_rows.size(); ++i) {
        const size_t v = vertex_rows[i];
        component_ids[i] = static_cast<double>(input.vertex_component_ids.at(v));
        singular[i] = input.singular_vertex_tags.at(v);
        normals.row(i) = input.optimal_normals.row(v);
        targets.row(i) = input.target_positions.row(v);
    }
    writer.add_field("component_id", component_ids);
    writer.add_field("singular_component", singular);
    writer.add_field("optimal_normal", normals);
    writer.add_field("target_position", targets);
}
} // namespace

void write_prismatic_mesh(const PrismaticMeshInput& input, const std::filesystem::path& path)
{
    const auto active_vertices = input.mesh->get_vertices();
    const size_t n = active_vertices.size();
    MatrixXd vertices(n, 3);
    VectorXd tags(n), ids(n), correspondence(n);
    std::vector<int> output_index(input.vertices.rows(), -1);
    std::vector<size_t> vertex_rows;
    for (size_t i = 0; i < n; ++i) {
        const size_t v = active_vertices[i].vid(*input.mesh);
        vertex_rows.push_back(v);
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
    VectorXd input_tags(tetrahedra.rows()), offset_tags(tetrahedra.rows()),
        tau22(tetrahedra.rows());
    for (Eigen::Index i = 0; i < tetrahedra.rows(); ++i) {
        input_tags[i] = input.input_cells[i];
        offset_tags[i] = input.offset_tet_tags[i];
        tau22[i] = classify_tau22(input, i) ? 1 : -1;
    }
    paraviewo::VTUWriter writer;
    writer.add_field("labels", tags);
    writer.add_field("vid", ids);
    writer.add_field("corr_input_vid", correspondence);
    writer.add_cell_field("tag_0", input_tags);
    writer.add_cell_field("offset_tag", offset_tags);
    writer.add_cell_field("tau22", tau22);
    add_targets(writer, input, vertex_rows);
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    if (!writer.write_mesh(path.string(), vertices, tetrahedra, paraviewo::CellType::Tetrahedron)) {
        log_and_throw_error("Could not write result mesh: {}", path.string());
    }
    // VTU CellData on the volume describes tetrahedra. Export face tags on a triangle mesh.
    if (!input.offset_face_tags.empty()) {
        std::vector<std::array<int, 3>> faces;
        std::vector<int> face_tags;
        std::vector<size_t> surface_vertices;
        std::vector<int> surface_index(input.vertices.rows(), -1);
        for (const auto& face : input.mesh->get_faces()) {
            const int tag = input.offset_face_tags.at(face.fid(*input.mesh));
            if (tag == -1) continue;
            const auto fv = input.mesh->get_face_vertices(face);
            std::array<int, 3> triangle;
            for (size_t j = 0; j < 3; ++j) {
                const size_t v = fv[j].vid(*input.mesh);
                if (surface_index[v] == -1) {
                    surface_index[v] = static_cast<int>(surface_vertices.size());
                    surface_vertices.push_back(v);
                }
                triangle[j] = surface_index[v];
            }
            faces.push_back(triangle);
            face_tags.push_back(tag);
        }
        MatrixXd surface_points(surface_vertices.size(), 3);
        VectorXd surface_labels(surface_vertices.size()), surface_ids(surface_vertices.size()),
            surface_corr(surface_vertices.size());
        for (size_t i = 0; i < surface_vertices.size(); ++i) {
            const auto v = surface_vertices[i];
            surface_points.row(i) = input.vertices.row(v);
            surface_labels[i] = input.vertex_tags[v];
            surface_ids[i] = static_cast<double>(input.source_vertex_ids[v]);
            surface_corr[i] = static_cast<double>(input.corr_input_vid[v]);
        }
        MatrixXi triangles(faces.size(), 3);
        VectorXd classification(faces.size());
        for (size_t i = 0; i < faces.size(); ++i) {
            for (int j = 0; j < 3; ++j) triangles(i, j) = faces[i][j];
            classification[i] = face_tags[i];
        }
        paraviewo::VTUWriter surface_writer;
        surface_writer.add_field("labels", surface_labels);
        surface_writer.add_field("vid", surface_ids);
        surface_writer.add_field("corr_input_vid", surface_corr);
        surface_writer.add_cell_field("offset_face_tag", classification);
        add_targets(surface_writer, input, surface_vertices);
        const auto surface_path = path.parent_path() / (path.stem().string() + "_offset_faces.vtu");
        if (!surface_writer.write_mesh(
                surface_path.string(),
                surface_points,
                triangles,
                paraviewo::CellType::Triangle)) {
            log_and_throw_error("Could not write offset faces: {}", surface_path.string());
        }
        logger().info("Wrote classified offset faces: {}", surface_path.string());
    }
}

} // namespace wmtk::components::prismatic_mesh
