#pragma once

#include <filesystem>
#include <wmtk/Types.hpp>

namespace wmtk::components::image_simulation {

class ImageSimulationMesh;

void extract_triangle_soup_from_image(std::string filename, Eigen::MatrixXi& F, Eigen::MatrixXd& V);

void tag_tets_from_image(const std::string& filename, ImageSimulationMesh& mesh);

} // namespace wmtk::components::image_simulation