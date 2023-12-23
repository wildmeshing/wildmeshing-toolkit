#pragma once

#include <filesystem>
#include <memory>

namespace wmtk {

class Mesh;

std::shared_ptr<Mesh> read_mesh(const std::filesystem::path& filename, const bool ignore_z = false);

} // namespace wmtk
