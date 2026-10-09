#include "background_remeshing.hpp"
#include "prismatic_mesh.hpp"

#include <algorithm>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {
namespace {
struct PreservedCell
{
    std::array<size_t, 4> vertices;
    int input_tag;
    int offset_tag;
    size_t original_tet_id;
};

void append_cells(PrismaticMeshInput& input, const std::vector<PreservedCell>& cells)
{
    if (cells.empty()) return;
    const size_t count = input.tetrahedra.rows();
    input.tetrahedra.conservativeResize(count + cells.size(), 4);
    for (size_t i = 0; i < cells.size(); ++i) {
        for (int j = 0; j < 4; ++j)
            input.tetrahedra(count + i, j) = static_cast<int>(cells[i].vertices[j]);
        input.input_cells.push_back(cells[i].input_tag);
        input.offset_tet_tags.push_back(cells[i].offset_tag);
    }
    std::vector<std::array<size_t, 4>> tets(input.tetrahedra.rows());
    for (size_t i = 0; i < tets.size(); ++i)
        for (int j = 0; j < 4; ++j) tets[i][j] = static_cast<size_t>(input.tetrahedra(i, j));
    auto mesh = std::make_unique<TetMesh>();
    mesh->init_with_isolated_vertices(input.vertices.rows(), tets);
    input.mesh = std::move(mesh);
    input.offset_face_tags.clear();
}
} // namespace

void keep_offset_band(PrismaticMeshInput& input)
{
    const auto count = std::count(input.offset_tet_tags.begin(), input.offset_tet_tags.end(), 1);
    MatrixXi tetrahedra(count, 4);
    std::vector<std::array<size_t, 4>> tets;
    std::vector<int> input_cells;
    std::vector<int> offset_tet_tags;
    tets.reserve(count);
    input_cells.reserve(count);
    offset_tet_tags.reserve(count);
    for (Eigen::Index i = 0; i < input.tetrahedra.rows(); ++i) {
        if (input.offset_tet_tags[i] != 1) continue;
        tetrahedra.row(tets.size()) = input.tetrahedra.row(i);
        std::array<size_t, 4> tet;
        for (int j = 0; j < 4; ++j) tet[j] = static_cast<size_t>(input.tetrahedra(i, j));
        tets.push_back(tet);
        input_cells.push_back(input.input_cells[i]);
        offset_tet_tags.push_back(input.offset_tet_tags[i]);
    }
    auto mesh = std::make_unique<TetMesh>();
    mesh->init_with_isolated_vertices(input.vertices.rows(), tets);
    input.mesh = std::move(mesh);
    input.tetrahedra = std::move(tetrahedra);
    input.input_cells = std::move(input_cells);
    input.offset_tet_tags = std::move(offset_tet_tags);
    input.offset_face_tags.clear();
    input.offset_components.clear();
    input.input_to_components.clear();
    input.vertex_component_ids.clear();
    input.singular_vertex_tags.clear();
    input.optimal_normals.resize(0, 3);
    input.target_positions.resize(0, 3);
    input.input_average_edge_length = 0;
    input.target_thickness = 0;
    input.optimization_iterations.clear();
    input.jacobian_smoothing_report = nullptr;
    input.background_remeshing_report = nullptr;
    input.fixed_background_vertices.clear();
}

void label_offset_faces(PrismaticMeshInput& input)
{
    input.offset_face_tags.assign(4 * input.mesh->tet_capacity(), -1);
    std::array<size_t, 3> counts = {0, 0, 0};
    for (const auto& face : input.mesh->get_faces()) {
        const auto vertices = input.mesh->get_face_vertices(face);
        std::array<int64_t, 3> correspondence;
        bool is_offset = true;
        for (size_t j = 0; j < 3; ++j) {
            const size_t v = vertices[j].vid(*input.mesh);
            if (input.vertex_tags.at(v) != 2) {
                is_offset = false;
                break;
            }
            correspondence[j] = input.corr_input_vid.at(v);
        }
        if (!is_offset) continue;
        for (const auto id : correspondence) {
            if (id < 0) log_and_throw_error("Offset face has a vertex without correspondence.");
        }
        const auto a = correspondence[0], b = correspondence[1], c = correspondence[2];
        const int tag = (a == b && b == c) ? 3 : ((a == b || b == c || a == c) ? 2 : 1);
        input.offset_face_tags[face.fid(*input.mesh)] = tag;
        ++counts[tag - 1];
    }
    logger().info(
        "Offset faces: {} bijective (1), {} with two equal correspondences (2), {} with all equal "
        "(3)",
        counts[0],
        counts[1],
        counts[2]);
}

void prism_main(
    PrismaticMeshInput& input,
    double thicknessratio,
    const OptimizationOptions& optimization)
{
    validate_background_remeshing_options(optimization.background_remeshing);
    if (optimization.background_remeshing.enabled && !optimization.keep_background_mesh)
        log_and_throw_error("background_remeshing requires keep_background_mesh=true.");
    // With remeshing enabled all background stars must be present, including neighbors
    // of movable interior vertices. Otherwise only offset-incident cells can change.
    // In the default mode, cutting away far background creates no boundary face incident
    // to an offset vertex, so its link/volume constraints match the full domain.
    std::vector<PreservedCell> preserved_input_tets, background_tets, fixed_background_tets;
    for (Eigen::Index i = 0; i < input.tetrahedra.rows(); ++i) {
        if (input.offset_tet_tags[i] == 1) continue;
        PreservedCell cell;
        for (int j = 0; j < 4; ++j) cell.vertices[j] = static_cast<size_t>(input.tetrahedra(i, j));
        cell.input_tag = input.input_cells[i];
        cell.offset_tag = input.offset_tet_tags[i];
        cell.original_tet_id = static_cast<size_t>(i);
        if (cell.input_tag == 1)
            preserved_input_tets.push_back(cell);
        else if (optimization.keep_background_mesh) {
            const bool adjacent =
                optimization.background_remeshing.enabled ||
                std::any_of(cell.vertices.begin(), cell.vertices.end(), [&](size_t v) {
                    return input.vertex_tags[v] == 2;
                });
            (adjacent ? background_tets : fixed_background_tets).push_back(cell);
        }
    }
    keep_offset_band(input);
    size_t active_input = 0, active_offset = 0, two_input_two_offset = 0, tau22 = 0;
    for (const auto& vertex : input.mesh->get_vertices()) {
        const int tag = input.vertex_tags.at(vertex.vid(*input.mesh));
        if (tag == 1) ++active_input;
        if (tag == 2) ++active_offset;
    }
    for (const auto& tet : input.mesh->get_tets()) {
        size_t in = 0, off = 0;
        for (size_t v : input.mesh->oriented_tet_vids(tet)) {
            if (input.vertex_tags.at(v) == 1) ++in;
            if (input.vertex_tags.at(v) == 2) ++off;
        }
        if (in == 2 && off == 2) ++two_input_two_offset;
        if (classify_tau22(input, tet.tid(*input.mesh))) ++tau22;
    }
    logger().info(
        "Kept offset band: {} tetrahedra, {} active vertices ({} input, {} offset); "
        "{} input-labeled vertices outside the band",
        input.tetrahedra.rows(),
        input.mesh->get_vertices().size(),
        active_input,
        active_offset,
        std::count(input.vertex_tags.begin(), input.vertex_tags.end(), 1) - active_input);
    logger().info(
        "Initial band: {} tetrahedra with 2 input + 2 offset vertices; {} tau22 with "
        "one-to-one correspondence to those same input vertices",
        two_input_two_offset,
        tau22);
    label_offset_faces(input);
    evaluate_target_positions(input, thicknessratio);
    input.fixed_background_vertices.assign(input.vertices.rows(), false);
    for (const auto& cell : fixed_background_tets) {
        for (size_t v : cell.vertices) input.fixed_background_vertices[v] = true;
        if (optimization.iterations > 0 &&
            !tet_volume_above_threshold(input.vertices, cell.vertices, 0)) {
            log_and_throw_error(
                "Initial fixed background tetrahedron {} has nonpositive or nonfinite volume.",
                cell.original_tet_id);
        }
    }
    append_cells(input, background_tets);
    if (!background_tets.empty()) label_offset_faces(input);
    logger().info(
        "Optimization domain: {} offset band tetrahedra, {} adjacent background tetrahedra; "
        "{} fixed background tetrahedra stored separately "
        "(keep_background_mesh={}, background_remeshing={})",
        std::count(input.offset_tet_tags.begin(), input.offset_tet_tags.end(), 1),
        background_tets.size(),
        fixed_background_tets.size(),
        optimization.keep_background_mesh,
        optimization.background_remeshing.enabled);
    optimize_prismatic_mesh(input, optimization);
    smooth_prism_jacobians(input, optimization.min_tet_volume, optimization.jacobian_smoothing);
    fixed_background_tets.insert(
        fixed_background_tets.end(),
        preserved_input_tets.begin(),
        preserved_input_tets.end());
    append_cells(input, fixed_background_tets);
    input.fixed_background_vertices.clear();
    if (!fixed_background_tets.empty()) label_offset_faces(input);
    size_t background_count = 0;
    for (size_t i = 0; i < input.offset_tet_tags.size(); ++i)
        if (input.offset_tet_tags[i] != 1 && input.input_cells[i] != 1) ++background_count;
    logger().info(
        "Output volume: {} tetrahedra ({} input, {} offset band, {} background); "
        "preserving all {} input vertices",
        input.tetrahedra.rows(),
        std::count(input.input_cells.begin(), input.input_cells.end(), 1),
        std::count(input.offset_tet_tags.begin(), input.offset_tet_tags.end(), 1),
        background_count,
        std::count(input.vertex_tags.begin(), input.vertex_tags.end(), 1));
}

} // namespace wmtk::components::prismatic_mesh
