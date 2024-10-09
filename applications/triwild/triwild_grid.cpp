#include "triwild_grid.hpp"

#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::triwild {

wmtk::TriMesh generate_bg_grid(
    const double x_min,
    const double y_min,
    const double x_max,
    const double y_max,
    const double length_rel,
    const double margin_eps)
{
    const double input_diag =
        (Eigen::Vector2d(x_min, y_min) - Eigen::Vector2d(x_max, y_max)).norm();
    const double target_length = length_rel * input_diag;
    const double y_start = y_min - input_diag * margin_eps;
    const double x_start = x_min - input_diag * margin_eps;
    const double y_end = y_max + input_diag * margin_eps;
    const double x_end = x_max + input_diag * margin_eps;

    const int64_t x_grid_size = floor((x_end - x_start) / target_length);
    const int64_t y_grid_size = floor((y_end - y_start) / target_length);
    const double x_space = (x_end - x_start) / x_grid_size;
    const double y_space = (y_end - y_start) / y_grid_size;

    Eigen::MatrixX<Rational> V;
    V.resize((x_grid_size + 1) * (y_grid_size + 1), 2);

    for (int64_t i = 0; i < y_grid_size + 1; ++i) {
        for (int64_t j = 0; j < x_grid_size + 1; ++j) {
            // V(i * (x_grid_size + 1) + j, 0) = x_min + j * x_space;
            // V(i * (x_grid_size + 1) + j, 1) = y_min + i * y_space;
            V.row(i * (x_grid_size + 1) + j) << x_start + j * x_space, y_start + i * y_space;
        }
    }

    RowVectors3l F;
    F.resize(x_grid_size * y_grid_size * 2, 3);

    for (int64_t i = 0; i < y_grid_size; ++i) {
        for (int64_t j = 0; j < x_grid_size; ++j) {
            F.row((i * x_grid_size + j) * 2) << i * (x_grid_size + 1) + j,
                i * (x_grid_size + 1) + j + 1, (i + 1) * (x_grid_size + 1) + j + 1;

            // std::cout << (i * x_grid_size + j) * 2 << ": " << i * x_grid_size + j << " "
            //           << i * x_grid_size + j + 1 << " " << (i + 1) * x_grid_size + j + 1
            //           << std::endl;

            F.row((i * x_grid_size + j) * 2 + 1) << i * (x_grid_size + 1) + j,
                (i + 1) * (x_grid_size + 1) + j + 1, (i + 1) * (x_grid_size + 1) + j;
        }
    }

    std::cout << F << std::endl;

    wmtk::TriMesh mesh;
    mesh.initialize(F, false);
    wmtk::mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, mesh);

    return mesh;
}

} // namespace wmtk::triwild