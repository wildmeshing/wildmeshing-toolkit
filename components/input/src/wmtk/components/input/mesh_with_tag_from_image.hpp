#pragma once

#include <wmtk/TriMesh.hpp>

#include <filesystem>
namespace wmtk::components::input {


/**
 * @brief Build a tagged triangle mesh from a greyscale image.
 *
 * For each pixel, two triangles are generated. The function was only tested on .png but might also
 * support .jpg. Support for .exr and .tif might be added in the future.
 *
 * @param file path to the image
 * @param tag_name attribute name for the face tag representing the images greyscale value
 */
std::shared_ptr<wmtk::TriMesh> mesh_with_tag_from_image(
    const std::filesystem::path& file,
    const std::string& tag_name);
} // namespace wmtk::components::input
