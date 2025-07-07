#pragma once

#include <filesystem>

namespace wmtk::components::image_simulation {

void raw_to_tetmesh(
    const std::filesystem::path& input_file,
    const std::filesystem::path& output_file);

}