#include "input.hpp"

#include <wmtk/io/read_mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "InputOptions.hpp"

namespace wmtk::components::input {

std::shared_ptr<Mesh> input(
    const std::filesystem::path& file,
    const bool ignore_z_if_zero,
    const std::vector<std::string>& tetrahedron_attributes)
{
    InputOptions options;
    options.old_mode = true;
    options.file = file;
    options.ignore_z_if_zero = ignore_z_if_zero;
    if (!tetrahedron_attributes.empty()) {
        options.imported_attributes = {{}, {}, {}, tetrahedron_attributes};
    }
    return input(options).root().shared_from_this();
}

NamedMultiMesh input(const InputOptions& options)
{
    std::filesystem::path file = options.file;
    if(options.working_directory.has_value() && file.is_relative()) {
        file = options.working_directory.value() / file;
    }

    if (!std::filesystem::exists(file)) {
        log_and_throw_error("file [{}] not found", file.string());
    }

    std::shared_ptr<Mesh> mesh;

    if (options.old_mode) {
        if (options.imported_attributes.has_value()) {
            mesh = wmtk::io::read_mesh(
                options.file,
                options.ignore_z_if_zero,
                options.imported_attributes->at(3));
        } else {
            mesh = wmtk::io::read_mesh(options.file, options.ignore_z_if_zero);
        }
    } else {
        if (options.imported_attributes.has_value()) {
            mesh = wmtk::io::read_mesh(file, options.imported_attributes.value());
        } else {
            mesh = wmtk::io::read_mesh(file);
        }
    }
    assert(mesh->is_connectivity_valid());


    NamedMultiMesh mm;
    mm.set_mesh(*mesh);
    mm.set_names(options.name_spec);

    return mm;
}
} // namespace wmtk::components::input
