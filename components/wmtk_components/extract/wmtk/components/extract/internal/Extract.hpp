#pragma once

#include <wmtk/TriMesh.hpp>

#include <filesystem>
namespace wmtk::components::internal {
void extract_triangle_soup_from_image(std::string output_path, std::string filename);
void read_array_data(
    std::vector<std::vector<std::vector<unsigned int>>>& data,
    const std::string& filename);
} // namespace wmtk::components::internal
