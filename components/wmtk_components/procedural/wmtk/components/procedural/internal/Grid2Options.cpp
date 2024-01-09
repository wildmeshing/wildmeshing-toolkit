#include "Grid2Options.hpp"
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "grid_utils.hpp"


namespace wmtk::components::internal {
const std::array<std::string, 2> Grid2Options::tiling_names = {{"bcc", "diagonal"}};

namespace {
std::shared_ptr<Mesh> make_diagonal_mesh(const Grid2Options& opt)
{
    const auto& d = opt.dimensions;
    auto vertex_dimensions = opt.dimensions;
    for (int64_t& v : vertex_dimensions) {
        v++;
    }
    using CoordType = std::array<int64_t, 2>;

    Eigen::Matrix<int64_t, Eigen::Dynamic, 3> FV(2 * d[0] * d[1], 3);

    CoordType i;
    //auto& [j, k] = i;
    int64_t& j = i[0];
    int64_t& k = i[1];
    for (j = 0; j < d[0]; ++j) {
        for (k = 0; k < d[1]; ++k) {
            auto f = [&](int64_t a, int64_t b) {
                return procedural::grid_index(vertex_dimensions, CoordType{{j + a, k + b}});
            };
            int64_t c[2][2] = {
                {
                    f(0, 0),
                    f(1, 0),
                },
                {
                    f(0, 1),
                    f(1, 1),
                }};
            int64_t f0_index = 2 * procedural::grid_index(d, i);
            int64_t f1_index = f0_index + 1;

            FV.row(f0_index) << c[0][0], c[0][1], c[1][0];
            FV.row(f1_index) << c[1][1], c[0][1], c[1][0];
        }
    }

    auto m = std::make_shared<TriMesh>();
    m->initialize(FV);
    if (opt.coordinates.has_value()) {
        int vertex_size = vertex_dimensions[0] * vertex_dimensions[1];

        const auto& spacing = opt.coordinates->spacing;
        Eigen::Matrix<double, Eigen::Dynamic, 2> P(vertex_size, 2);
        for (j = 0; j < vertex_dimensions[0]; ++j) {
            double x = spacing[0] * j;
            for (k = 0; k < vertex_dimensions[1]; ++k) {
                double y = spacing[1] * k;
                int64_t index = procedural::grid_index(vertex_dimensions, i);

                P.row(index) << x, y;
            }
        }
        const auto& name = opt.coordinates->name;
        wmtk::mesh_utils::set_matrix_attribute(P, name, PrimitiveType::Vertex, *m);
    }
    return m;
}
} // namespace

namespace procedural {
std::shared_ptr<Mesh> make_mesh(const Grid2Options& opt)
{
    switch (opt.tiling_type) {
    case Grid2Options::TilingType::Diagonal: return make_diagonal_mesh(opt);
    case Grid2Options::TilingType::BCC:
        throw std::runtime_error("bcc lattice not implemented yet");
        [[fallthrough]];
    default: break;
    }
    throw std::runtime_error("failed to select a tiling type");
    return nullptr;
}
} // namespace procedural
} // namespace wmtk::components::internal
