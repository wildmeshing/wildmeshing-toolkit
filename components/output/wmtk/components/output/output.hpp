#pragma once

#include <wmtk/Mesh.hpp>

#include <filesystem>


namespace wmtk::components {

void output(
    const Mesh& mesh,
    const std::filesystem::path& file,
    const std::string& position_attr_name = "vertices");

} // namespace wmtk::components