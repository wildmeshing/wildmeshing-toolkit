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
    InputOptions options;
    options.old_mode = true;
    options.file = file;
    if (!tetrahedron_attributes.empty()) {
        options.imported_attributes = {{}, {}, {}, tetrahedron_attributes};
    }
    return input(options).root().shared_from_this();
}

NamedMultiMesh input(const InputOptions& options)
{
    if (!std::filesystem::exists(options.file)) {
        log_and_throw_error("file [{}] not found", options.file.string());
    }

    std::shared_ptr<Mesh> mesh;

    if (options.old_mode) {
        if (options.imported_attributes.has_value()) {
            mesh = wmtk::io::read_mesh(
                options.file,
                options.ignore_z,
                options.imported_attributes->at(3));
        } else {
            mesh = wmtk::io::read_mesh(options.file, options.ignore_z);
        }
    } else {
        if (options.imported_attributes.has_value()) {
            mesh = wmtk::io::read_mesh(options.file, options.imported_attributes.value());
        } else {
            mesh = wmtk::io::read_mesh(options.file);
        }
    }
    assert(mesh->is_connectivity_valid());


    NamedMultiMesh mm;
    mm.set_mesh(*mesh);
    mm.set_names(options.name_spec);

    return mm;
}
} // namespace wmtk::components::input
