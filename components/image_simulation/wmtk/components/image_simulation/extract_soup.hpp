#pragma once

#include <filesystem>
#include <wmtk/Types.hpp>

namespace wmtk::components::image_simulation {

class ImageSimulationMesh;

void read_array_data_ascii(
    std::vector<std::vector<std::vector<size_t>>>& data,
    const std::string& filename);

void extract_triangle_soup_from_image(std::string filename, Eigen::MatrixXi& F, Eigen::MatrixXd& V);

/**
 * @brief Extracts a triangle soup from a volumetric image.
 *
 * Whenever two voxels contain different data, two triangles are inserted in between the two voxels.
 *
 * @param data The voxel image data.
 * @param F #Fx3 triangles.
 * @param V #Vx3 vertices.
 * @param F_tags #Fx2 The voxel data that was separated by this triangle.
 */
void extract_triangle_soup_from_image(
    const std::vector<std::vector<std::vector<size_t>>>& data,
    MatrixXi& F,
    MatrixXd& V,
    MatrixXi& F_tags);

void tag_tets_from_image(const std::string& filename, ImageSimulationMesh& mesh);

void image_to_tagged_tets(const std::string& filename, MatrixXd& V, MatrixXi& T, VectorXi T_tags);

} // namespace wmtk::components::image_simulation