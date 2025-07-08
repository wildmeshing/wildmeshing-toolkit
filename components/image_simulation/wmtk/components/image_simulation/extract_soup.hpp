#pragma once

#include <filesystem>
#include <wmtk/Types.hpp>

namespace wmtk::components::image_simulation {

void extract_triangle_soup_from_image(std::string filename, Eigen::MatrixXi& F, Eigen::MatrixXd& V);

} // namespace wmtk::components::image_simulation