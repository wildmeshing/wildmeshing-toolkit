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
}

void prism_main(PrismaticMeshInput& input)
{
    keep_offset_band(input);
    logger().info(
        "Kept offset band: {} tetrahedra, {} active vertices",
        input.tetrahedra.rows(),
        input.mesh->get_vertices().size());
    // The band mesh and correspondence are ready for subsequent prism construction here.
}

} // namespace wmtk::components::prismatic_mesh
