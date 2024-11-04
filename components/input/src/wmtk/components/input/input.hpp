#pragma once

#include <filesystem>

#include <wmtk/Mesh.hpp>

#include "NamedMultiMesh.hpp"
namespace wmtk::components::input {
class InputOptions;


/*
 * @brief Read a mesh from file.
 *
 * This method wraps MeshReader::read_mesh and adds some validity checks.
 * It can read .msh and .hdf5 file format.
 *
 * @param file The mesh file.
 * @param ignore_z_if_zero Ignore the z-component of points if it is zero and generate a mesh with
 * 2D positions.
 * @param tetrahedron_attributes Read tetrahedron attributes from an .msh file.
 */
std::shared_ptr<Mesh> input(
    const std::filesystem::path& file,
    const bool ignore_z_if_zero = false,
    const std::vector<std::string>& tetrahedron_attributes = {});

NamedMultiMesh input(const InputOptions& options);

} // namespace wmtk::components::input
