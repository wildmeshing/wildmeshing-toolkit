#include "input.hpp"

#include <wmtk/io/read_mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "InputOptions.hpp"

namespace wmtk::components::input {

std::shared_ptr<Mesh> input(
    const std::filesystem::path& file,
    const bool ignore_z,
    const std::vector<std::string>& tetrahedron_attributes)
{
    if (!std::filesystem::exists(file)) {
        log_and_throw_error("file {} not found", file.string());
    }

    const std::shared_ptr<Mesh> mesh = wmtk::io::read_mesh(file, ignore_z, tetrahedron_attributes);
    assert(mesh->is_connectivity_valid());

    return mesh;
}

std::shared_ptr<Mesh> input(const InputOptions& options)
{
    if (!std::filesystem::exists(options.file)) {
        log_and_throw_error("file {} not found", options.file.string());
    }

    std::shared_ptr<Mesh> mesh;
    if (options.imported_attributes.has_value()) {
        mesh = wmtk::io::read_mesh(options.file, options.imported_attributes.value());
    } else {
        mesh = wmtk::io::read_mesh(options.file);
    }
    assert(mesh->is_connectivity_valid());

    return mesh;
}
} // namespace wmtk::components::input
