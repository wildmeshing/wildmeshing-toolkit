#pragma once
#include <spdlog/spdlog.h>
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <filesystem>
#define TINYEXR_USE_MINIZ 0
#define TINYEXR_USE_STB_ZLIB 1
// #define TINYEXR_IMPLEMENTATION
#include <tinyexr.h>
#include <cassert>
#include "Logger.hpp"
namespace wmtk {

bool save_image_exr_red_channel(
    size_t weigth,
    size_t height,
    const std::vector<float>& data,
    const std::filesystem::path& path);

}