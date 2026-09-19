#include "prismatic_mesh.hpp"

#include <algorithm>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {

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
    keep_offset_band(input);
    logger().info(
        "Kept offset band: {} tetrahedra, {} active vertices",
        input.tetrahedra.rows(),
        input.mesh->get_vertices().size());
    label_offset_faces(input);
    evaluate_target_positions(input, thicknessratio);
    optimize_prismatic_mesh(input, optimization);
}

} // namespace wmtk::components::prismatic_mesh
