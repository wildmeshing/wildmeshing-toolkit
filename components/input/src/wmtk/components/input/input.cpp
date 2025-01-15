#include "input.hpp"

#include <fstream>
#include <wmtk/components/utils/PathResolver.hpp>
#include <wmtk/io/read_mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "InputOptions.hpp"
#include <wmtk/utils/verify_simplex_index_valences.hpp>

namespace wmtk::components::input {

std::shared_ptr<Mesh> input(
    const std::filesystem::path& file,
    const bool ignore_z_if_zero,
    const std::vector<std::string>& tetrahedron_attributes)
{
    InputOptions options;
    options.old_mode = true;
    options.path = file;
    options.ignore_z_if_zero = ignore_z_if_zero;
    if (!tetrahedron_attributes.empty()) {
        options.imported_attributes = {{}, {}, {}, tetrahedron_attributes};
    }
    return input(options).root().shared_from_this();
}


multimesh::NamedMultiMesh input(const InputOptions& options)
{
    return input(options, {});
}
multimesh::NamedMultiMesh input(
    const InputOptions& options,
    const components::utils::PathResolver& resolver)
{
    const auto [file_path, found] = resolver.resolve(options.path);
    if (!found) {
        const auto& paths = resolver.get_paths();
        std::vector<std::string> path_strs;
        std::transform(
            paths.begin(),
            paths.end(),
            std::back_inserter(path_strs),
            [](const std::filesystem::path& p) { return p.string(); });

        log_and_throw_error(
            "file [{}] not found (input path was [{}], paths searched were [{}]",
            file_path.string(),
            options.path.string(),
            fmt::join(path_strs, ","));
    }


    std::shared_ptr<Mesh> mesh;

    if (options.old_mode) {
        if (options.imported_attributes.has_value()) {
            mesh = wmtk::io::read_mesh(
                file_path,
                options.ignore_z_if_zero,
                options.imported_attributes->at(3));
        } else {
            mesh = wmtk::io::read_mesh(file_path, options.ignore_z_if_zero);
        }
    } else {
        if (options.imported_attributes.has_value()) {
            mesh = wmtk::io::read_mesh(file_path, options.imported_attributes.value());
        } else {
            mesh = wmtk::io::read_mesh(file_path);
        }
    }
    assert(mesh->is_connectivity_valid());


    multimesh::NamedMultiMesh mm;
    mm.set_mesh(*mesh);
    if (!options.name_spec.is_null()) {
        mm.set_names(options.name_spec);
    } else if (options.name_spec_file.has_value()) {
        std::ifstream ifs(options.name_spec_file.value());
        nlohmann::json js;
        ifs >> js;
        mm.set_names(js);
    }

    if(options.validate) {
        for(auto& mptr: mm.root().get_all_meshes()) {
            if(!mm.has_name(*mptr) && !mptr->is_multi_mesh_root()) {

                mptr->get_multi_mesh_parent_mesh().deregister_child_mesh(mptr);

            }
        }
    }

    if(options.validate) {
        for(const auto& mptr: mm.root().get_all_meshes()) {
            if(!wmtk::utils::verify_simplex_index_valences(*mptr)) {
                throw std::runtime_error(fmt::format("Mesh {} was not valid, check env WMTK_LOGGER_LEVEL=debug for more info", mm.get_name(*mptr)));
            }
        }

    }

    return mm;
}
} // namespace wmtk::components::input
