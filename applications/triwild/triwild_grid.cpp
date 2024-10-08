#include "triwild_grid.hpp"

namespace wmtk::triwild {

std::shared_ptr<TriMesh> generate_bg_grid(
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
    
}

} // namespace wmtk::triwild