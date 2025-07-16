#pragma once

#include <filesystem>
#include <wmtk/Types.hpp>

namespace wmtk::components::image_simulation {

class ImageSimulationMesh;

void read_array_data_ascii(
    std::vector<std::vector<std::vector<size_t>>>& data,
    const std::string& filename);

void extract_triangle_soup_from_image(std::string filename, Eigen::MatrixXi& F, Eigen::MatrixXd& V);

void tag_tets_from_image(const std::string& filename, ImageSimulationMesh& mesh);

void image_to_tagged_tets(const std::string& filename, MatrixXd& V, MatrixXi& T, VectorXi T_tags);

} // namespace wmtk::components::image_simulation