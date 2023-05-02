#pragma once
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <filesystem>
namespace wmtk {

auto load_image_exr_red_channel(const std::filesystem::path& path)
    -> std::tuple<size_t, size_t, std::vector<float>>;

}