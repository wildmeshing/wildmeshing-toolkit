#pragma once
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <filesystem>
namespace wmtk::components::adaptive_tessellation::image {

auto load_image_exr_red_channel(const std::filesystem::path& path)
    -> std::tuple<size_t, size_t, std::vector<float>>;

auto load_image_exr_split_3channels(const std::filesystem::path& path) -> std::tuple<
    size_t,
    size_t,
    int,
    int,
    int,
    std::vector<float>,
    std::vector<float>,
    std::vector<float>>;
} // namespace wmtk::components::adaptive_tessellation::image
