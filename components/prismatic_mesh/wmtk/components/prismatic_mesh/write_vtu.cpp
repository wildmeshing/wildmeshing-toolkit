#include "prismatic_mesh.hpp"
#include "write_msh.hpp"

#include <algorithm>
#include <fstream>
#include <paraviewo/VTUWriter.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {
namespace {
void add_targets(
    detail::MeshFields& fields,
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
    fields.emplace("component_id", std::move(component_ids));
    fields.emplace("singular_component", std::move(singular));
    fields.emplace("optimal_normal", std::move(normals));
    fields.emplace("target_position", std::move(targets));
}
} // namespace

void write_prismatic_mesh(
    const PrismaticMeshInput& input,
    const std::filesystem::path& path,
    const PrismDominantMesh* hybrid,
    bool include_background)
{
    if (path.extension() != ".vtu" && path.extension() != ".msh")
        log_and_throw_error("Prismatic mesh output must be a .vtu or .msh file.");
    const auto keep_tet = [&](size_t tid) {
        return include_background || input.input_cells.at(tid) == 1 ||
               input.offset_tet_tags.at(tid) == 1;
    };
    std::vector<size_t> cell_rows;
    const size_t full_cell_count = hybrid ? hybrid->cells.size() : input.tetrahedra.rows();
    for (size_t i = 0; i < full_cell_count; ++i)
        if (keep_tet(hybrid ? hybrid->cells[i].source_tets.at(0) : i)) cell_rows.push_back(i);
    std::vector<bool> retained(input.vertices.rows(), false);
    if (include_background) {
        for (const auto& vertex : input.mesh->get_vertices())
            retained[vertex.vid(*input.mesh)] = true;
    } else {
        for (size_t row : cell_rows) {
            if (hybrid) {
                for (size_t v : hybrid->cells[row].vertices) retained.at(v) = true;
            } else {
                for (int j = 0; j < 4; ++j) retained.at(input.tetrahedra(row, j)) = true;
            }
        }
    }
    std::vector<size_t> vertex_rows;
    for (size_t v = 0; v < retained.size(); ++v) {
        if (retained[v] || input.vertex_tags[v] == 1) vertex_rows.push_back(v);
    }
    const size_t n = vertex_rows.size();
    MatrixXd vertices(n, 3);
    VectorXd tags(n), ids(n), correspondence(n);
    std::vector<int> output_index(input.vertices.rows(), -1);
    for (size_t i = 0; i < n; ++i) {
        const size_t v = vertex_rows[i];
        output_index[v] = static_cast<int>(i);
        vertices.row(i) = input.vertices.row(v);
        tags[i] = input.vertex_tags[v];
        ids[i] = static_cast<double>(input.source_vertex_ids[v]);
        correspondence[i] = static_cast<double>(input.corr_input_vid[v]);
    }
    // Compact output row indices only. vid and corr_input_vid remain source IDs.
    const size_t nc = cell_rows.size();
    VectorXd input_tags(nc), offset_tags(nc), tau22(nc);
    VectorXd cell_types, source_counts, candidates, retriangulated;
    std::vector<bool> changed_reference_tets(input.tetrahedra.rows(), false);
    MatrixXd reference_tets, source_tet_ids;
    std::vector<paraviewo::CellElement> mixed_cells;
    mixed_cells.reserve(nc);
    if (hybrid) {
        cell_types.resize(nc);
        source_counts.resize(nc);
        candidates.resize(nc);
        retriangulated.resize(nc);
        if (hybrid->report.contains("retriangulated_regions"))
            for (const auto& region : hybrid->report["retriangulated_regions"])
                for (const auto& tid : region["reference_tet_rows"])
                    changed_reference_tets.at(tid.get<size_t>()) = true;
        reference_tets = MatrixXd::Constant(nc, 12, -1);
        source_tet_ids = MatrixXd::Constant(nc, 3, -1);
    }
    for (size_t i = 0; i < nc; ++i) {
        const size_t row = cell_rows[i];
        const size_t tid = hybrid ? hybrid->cells[row].source_tets.at(0) : row;
        input_tags[i] = input.input_cells[tid];
        offset_tags[i] = input.offset_tet_tags[tid];
        const bool is_tet = !hybrid || hybrid->cells[row].type == HybridCellType::Tetrahedron;
        tau22[i] = is_tet && classify_tau22(input, tid) ? 1 : -1;
        if (!hybrid) {
            paraviewo::CellElement out;
            out.ctype = paraviewo::CellType::Tetrahedron;
            for (int j = 0; j < 4; ++j) {
                const int v = output_index.at(input.tetrahedra(row, j));
                if (v < 0) log_and_throw_error("Output tetrahedron references an inactive vertex.");
                out.vertices.push_back(v);
            }
            mixed_cells.push_back(std::move(out));
            continue;
        }
        const auto& cell = hybrid->cells[row];
        if (cell.source_tets.size() > 3)
            log_and_throw_error("Invalid hybrid source tetrahedron count.");
        paraviewo::CellElement out;
        out.ctype = cell.type == HybridCellType::Prism     ? paraviewo::CellType::Wedge
                    : cell.type == HybridCellType::Pyramid ? paraviewo::CellType::Pyramid
                                                           : paraviewo::CellType::Tetrahedron;
        for (size_t v : cell.vertices) {
            if (output_index.at(v) < 0)
                log_and_throw_error("Hybrid cell references an inactive vertex.");
            out.vertices.push_back(output_index[v]);
        }
        mixed_cells.push_back(std::move(out));
        cell_types[i] = static_cast<int>(cell.type);
        source_counts[i] = cell.source_tets.size();
        candidates[i] = cell.prism_candidate;
        retriangulated[i] = std::any_of(
                                cell.source_tets.begin(),
                                cell.source_tets.end(),
                                [&](size_t tid) { return changed_reference_tets.at(tid); })
                                ? 1
                                : 0;
        for (size_t k = 0; k < cell.source_tets.size(); ++k) {
            source_tet_ids(i, k) = cell.source_tets[k];
            for (int j = 0; j < 4; ++j) {
                const size_t v = input.tetrahedra(cell.source_tets[k], j);
                const auto it = std::find(cell.vertices.begin(), cell.vertices.end(), v);
                if (it == cell.vertices.end())
                    log_and_throw_error("Invalid hybrid reference decomposition.");
                reference_tets(i, 4 * k + j) = std::distance(cell.vertices.begin(), it);
            }
        }
    }
    detail::MeshFields point_fields, cell_fields;
    point_fields.emplace("labels", std::move(tags));
    point_fields.emplace("vid", std::move(ids));
    point_fields.emplace("corr_input_vid", std::move(correspondence));
    cell_fields.emplace("tag_0", std::move(input_tags));
    cell_fields.emplace("offset_tag", std::move(offset_tags));
    cell_fields.emplace("tau22", std::move(tau22));
    if (hybrid) {
        cell_fields.emplace("cell_type", std::move(cell_types));
        cell_fields.emplace("source_tet_count", std::move(source_counts));
        cell_fields.emplace("prism_candidate", std::move(candidates));
        cell_fields.emplace("source_tet_ids", std::move(source_tet_ids));
        cell_fields.emplace("tet_decomposition", std::move(reference_tets));
        cell_fields.emplace("retriangulated", std::move(retriangulated));
    }
    add_targets(point_fields, input, vertex_rows);
    if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    if (path.extension() == ".msh") {
        detail::write_msh(path, vertices, mixed_cells, point_fields, cell_fields);
    } else {
        paraviewo::VTUWriter writer;
        for (const auto& [name, values] : point_fields) writer.add_field(name, values);
        for (const auto& [name, values] : cell_fields) writer.add_cell_field(name, values);
        if (!writer.write_mesh(path.string(), vertices, mixed_cells))
            log_and_throw_error("Could not write result mesh: {}", path.string());
    }
    if (hybrid) {
        const auto report_path = path.parent_path() / (path.stem().string() + "_hybrid.json");
        std::ofstream report(report_path);
        auto export_report = hybrid->report;
        if (!include_background) {
            size_t reference_tets = 0;
            for (size_t row : cell_rows) reference_tets += hybrid->cells[row].source_tets.size();
            export_report["export_scope"] = "input_and_shell";
            export_report["output_cells"] = nc;
            export_report["background_tetrahedra"] = 0;
            export_report["source_tetrahedra"] = reference_tets;
            export_report["full_reference_tetrahedra"] = input.tetrahedra.rows();
            export_report["source_tet_id_space"] = "full_reconstructed_mesh";
        }
        report << export_report.dump(2) << '\n';
        if (!report) log_and_throw_error("Could not write hybrid report: {}", report_path.string());
    }
    if (!input.background_remeshing_report.is_null()) {
        const auto report_path =
            path.parent_path() / (path.stem().string() + "_background_remeshing.json");
        std::ofstream report(report_path);
        report << input.background_remeshing_report.dump(2) << '\n';
        if (!report)
            log_and_throw_error(
                "Could not write background remeshing report: {}",
                report_path.string());
    }
    // VTU CellData on the volume describes volume cells. Export face tags on a triangle mesh.
    if (!input.offset_face_tags.empty()) {
        std::vector<std::array<int, 3>> faces;
        std::vector<int> face_tags;
        std::vector<size_t> surface_vertices;
        std::vector<int> surface_index(input.vertices.rows(), -1);
        for (const auto& face : input.mesh->get_faces()) {
            const int tag = input.offset_face_tags.at(face.fid(*input.mesh));
            if (tag == -1) continue;
            if (!keep_tet(face.tid(*input.mesh))) {
                const auto other = face.switch_tetrahedron(*input.mesh);
                if (!other || !keep_tet(other->tid(*input.mesh))) continue;
            }
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
        VectorXd classification(faces.size()), non_surjective(faces.size());
        size_t non_surjective_count = 0;
        for (size_t i = 0; i < faces.size(); ++i) {
            for (int j = 0; j < 3; ++j) triangles(i, j) = faces[i][j];
            classification[i] = face_tags[i];
            // Repeated correspondence IDs make this triangle map to an edge or vertex,
            // rather than cover a triangle with three distinct input vertices.
            const bool collapsed_image = face_tags[i] == 2 || face_tags[i] == 3;
            non_surjective[i] = collapsed_image ? 1 : 0;
            if (collapsed_image) ++non_surjective_count;
        }
        paraviewo::VTUWriter surface_writer;
        surface_writer.add_field("labels", surface_labels);
        surface_writer.add_field("vid", surface_ids);
        surface_writer.add_field("corr_input_vid", surface_corr);
        surface_writer.add_cell_field("offset_face_tag", classification);
        surface_writer.add_cell_field("non_surjective", non_surjective);
        detail::MeshFields surface_targets;
        add_targets(surface_targets, input, surface_vertices);
        for (const auto& [name, values] : surface_targets) surface_writer.add_field(name, values);
        const auto surface_path = path.parent_path() / (path.stem().string() + "_offset_faces.vtu");
        if (!surface_writer.write_mesh(
                surface_path.string(),
                surface_points,
                triangles,
                paraviewo::CellType::Triangle)) {
            log_and_throw_error("Could not write offset faces: {}", surface_path.string());
        }
        logger().info(
            "Wrote classified offset faces: {} ({} / {} non-surjective faces)",
            surface_path.string(),
            non_surjective_count,
            faces.size());
    }
}

} // namespace wmtk::components::prismatic_mesh
