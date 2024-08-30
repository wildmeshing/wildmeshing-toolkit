#pragma once

#include <nlohmann/json.hpp>
#include <wmtk/Mesh.hpp>

namespace wmtk::components {

// std::shared_ptr<Mesh> input(nlohmann::json j);

std::shared_ptr<Mesh> input(
    const std::filesystem::path& file,
    const bool ignore_z = false,
    const std::vector<std::string> tetrahedron_attributes = {});

} // namespace wmtk::components