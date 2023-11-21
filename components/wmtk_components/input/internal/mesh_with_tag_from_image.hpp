#pragma once

#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {


/**
 * @brief Build a tagged triangle mesh from a greyscale image.
 */
std::shared_ptr<wmtk::TriMesh> mesh_with_tag_from_image(
    const std::filesystem::path& file,
    const std::string& tag_name);
} // namespace wmtk::components::internal