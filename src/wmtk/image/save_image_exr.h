#pragma once
#include <spdlog/spdlog.h>
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <filesystem>
#define TINYEXR_USE_MINIZ 0
#define TINYEXR_USE_STB_ZLIB 1
// #define TINYEXR_IMPLEMENTATION
namespace wmtk {
namespace image {
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
} // namespace image
} // namespace wmtk