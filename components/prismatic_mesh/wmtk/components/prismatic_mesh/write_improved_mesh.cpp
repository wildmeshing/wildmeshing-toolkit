#include "quality_improvement.hpp"
#include "write_msh.hpp"

#include <fstream>
#include <limits>
#include <wmtk/io/DataWriter.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {
void write_improved_prismatic_mesh(
    const ImprovedHybridMesh& mesh,
    const std::filesystem::path& path,
    bool include_background,
    MeshMap* output,
    nlohmann::json* report)
{
    if (path.extension() != ".vtu" && path.extension() != ".msh")
        log_and_throw_error("Improved mesh output must be .vtu or .msh.");
    std::vector<size_t> rows;
    std::vector<bool> used(mesh.vertices.rows(), false);
    for (size_t i = 0; i < mesh.cells.size(); ++i) {
        if (!include_background && mesh.cells[i].region == 3) continue;
        rows.push_back(i);
        for (size_t v : mesh.cells[i].vertices) used.at(v) = true;
    }
    for (size_t v = 0; v < used.size(); ++v)
        if (mesh.vertex_tags.at(v) == 1) used[v] = true;
    // Retain discrete input anchors even if their stars disappeared during remeshing.
    for (size_t v = 0; v < used.size(); ++v)
        if (used[v] && mesh.correspondence.at(v) >= 0) used.at(mesh.correspondence[v]) = true;
    std::vector<size_t> vertices;
    std::vector<int64_t> index(mesh.vertices.rows(), -1);
    for (size_t v = 0; v < used.size(); ++v)
        if (used[v]) {
            index[v] = vertices.size();
            vertices.push_back(v);
        }
    const size_t nv = vertices.size(), nc = rows.size();
    MatrixXd points(nv, 3), base(nv, 3), normals(nv, 3);
    VectorXd labels(nv), ids(nv), corr(nv), original(nv), heights(nv), lengths(nv), projection(nv),
        error(nv), thickness_constrained(nv);
    DataMatrix<int64_t> integer_ids(nv, 1), integer_corr(nv, 1);
    // The ASCII MSH reader does not accept NaN tokens. Explicit validity makes
    // finite zeros unambiguous on unconstrained points in every export format.
    for (size_t i = 0; i < nv; ++i) {
        const size_t v = vertices[i];
        points.row(i) = mesh.vertices.row(v);
        base.row(i) = mesh.reference_base.row(v);
        normals.row(i) = mesh.reference_normals.row(v);
        labels[i] = mesh.vertex_tags[v];
        ids[i] = integer_ids(i, 0) = mesh.source_vertex_ids[v];
        const int64_t parent = mesh.correspondence[v];
        corr[i] = integer_corr(i, 0) = parent >= 0 ? mesh.source_vertex_ids.at(parent) : -1;
        original[i] = mesh.original_vertex_rows[v];
        heights[i] = mesh.target_heights[v];
        lengths[i] = mesh.target_edge_lengths[v];
        const bool constrained = labels[i] != 1 && heights[i] > 0 && normals.row(i).norm() > .5;
        thickness_constrained[i] = constrained;
        projection[i] = constrained ? (points.row(i) - base.row(i)).dot(normals.row(i)) : 0;
        error[i] = constrained ? projection[i] / heights[i] - 1 : 0;
    }
    VectorXd types(nc), regions(nc), input(nc), offset(nc), amips(nc), full_ids(nc);
    std::vector<paraviewo::CellElement> cells;
    for (size_t i = 0; i < nc; ++i) {
        const auto& c = mesh.cells[rows[i]];
        paraviewo::CellElement element;
        element.ctype = c.type == HybridCellType::Prism     ? paraviewo::CellType::Wedge
                        : c.type == HybridCellType::Pyramid ? paraviewo::CellType::Pyramid
                                                            : paraviewo::CellType::Tetrahedron;
        for (size_t v : c.vertices) element.vertices.push_back(index.at(v));
        // File VTU uses VTK winding. MeshData/Python and MSH use the same native
        // Gmsh winding as the construction exporter and the generic mesh I/O.
        if ((output || path.extension() == ".msh") && c.type == HybridCellType::Prism) {
            std::swap(element.vertices[1], element.vertices[2]);
            std::swap(element.vertices[4], element.vertices[5]);
        }
        cells.push_back(std::move(element));
        types[i] = static_cast<int>(c.type);
        regions[i] = c.region;
        input[i] = c.region == 1 ? 1 : 0;
        offset[i] = c.region == 2 ? 1 : -1;
        amips[i] = c.amips;
        full_ids[i] = rows[i];
    }
    detail::MeshFields point_fields, cell_fields;
    point_fields.emplace("labels", std::move(labels));
    point_fields.emplace("vid", std::move(ids));
    point_fields.emplace("corr_input_vid", std::move(corr));
    point_fields.emplace("original_vertex_row", std::move(original));
    point_fields.emplace("reference_base", std::move(base));
    point_fields.emplace("reference_normal", std::move(normals));
    point_fields.emplace("target_normal_thickness", std::move(heights));
    point_fields.emplace("target_edge_length", std::move(lengths));
    point_fields.emplace("projected_height", std::move(projection));
    point_fields.emplace("reference_height_error_ratio", std::move(error));
    point_fields.emplace("thickness_constrained", std::move(thickness_constrained));
    cell_fields.emplace("cell_type", std::move(types));
    cell_fields.emplace("region", std::move(regions));
    cell_fields.emplace("tag_0", std::move(input));
    cell_fields.emplace("offset_tag", std::move(offset));
    cell_fields.emplace("AMIPS_native", std::move(amips));
    cell_fields.emplace("full_mesh_cell_id", std::move(full_ids));
    if (!mesh.construction_regions.empty()) {
        VectorXd construction(nc), rod(nc), joint(nc);
        for (size_t i = 0; i < nc; ++i) {
            construction[i] = mesh.construction_regions.at(rows[i]);
            rod[i] = mesh.rod_regions.at(rows[i]);
            joint[i] = mesh.joint_one_ring.at(rows[i]);
        }
        cell_fields.emplace("construction_region", std::move(construction));
        cell_fields.emplace("rod_region", std::move(rod));
        cell_fields.emplace("joint_one_ring", std::move(joint));
    }
    if (!output && !path.parent_path().empty())
        std::filesystem::create_directories(path.parent_path());
    if (!output && path.extension() == ".msh")
        detail::write_msh(path, points, cells, point_fields, cell_fields);
    else {
        io::DataWriter writer(output);
        for (const auto& [name, values] : point_fields) writer.add_field(name, values);
        for (const auto& [name, values] : cell_fields) writer.add_cell_field(name, values);
        if (output) {
            writer.add_integer_field("vid", integer_ids);
            writer.add_integer_field("corr_input_vid", integer_corr);
        }
        if (!writer.write_mesh(path.string(), points, cells))
            log_and_throw_error("Could not write improved mesh: {}", path.string());
    }
    nlohmann::json exported = {
        {"quality_improvement", mesh.report},
        {"output_cells", nc},
        {"output_vertices", nv},
        {"export_scope", include_background ? "full" : "input_and_shell"},
        {"source_tet_id_space", "not_applicable_after_retopology"}};
    if (report) *report = exported;
    if (!output) {
        const auto file = path.parent_path() / (path.stem().string() + "_hybrid.json");
        std::ofstream stream(file);
        stream << exported.dump(2) << '\n';
        if (!stream)
            log_and_throw_error("Could not write quality improvement report: {}", file.string());
    }
}
} // namespace wmtk::components::prismatic_mesh
