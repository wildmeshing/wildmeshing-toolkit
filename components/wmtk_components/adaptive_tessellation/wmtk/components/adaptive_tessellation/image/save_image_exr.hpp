#pragma once

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <filesystem>
namespace wmtk::components::adaptive_tessellation::image {
bool save_image_exr_red_channel(
    size_t weigth,
    size_t height,
    const std::vector<float>& data,
    const std::filesystem::path& path);
bool save_image_exr_3channels(
    size_t width,
    size_t height,
    int r,
    int g,
    int b,
    const std::vector<float>& data_r,
    const std::vector<float>& data_g,
    const std::vector<float>& data_b,
    const std::filesystem::path& path);
} // namespace wmtk::components::adaptive_tessellation::image
