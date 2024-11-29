#include "output.hpp"

#include <fmt/std.h>
#include <type_traits>
#include <wmtk/Mesh.hpp>
#include <wmtk/components/multimesh/MeshCollection.hpp>
#include <wmtk/components/multimesh/NamedMultiMesh.hpp>
#include <wmtk/components/utils/resolve_path.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/Logger.hpp>
#include "OutputOptions.hpp"


namespace wmtk::components::output {

void output(
    const Mesh& mesh,
    const std::filesystem::path& file,
    const std::string& position_attr_name)
{
    wmtk::logger().info("Saving on {}", file.string());

    if (file.extension().empty()) {
        assert(!position_attr_name.empty());
        std::array<bool, 4> out = {{false, false, false, false}};
        for (int64_t d = 0; d <= mesh.top_cell_dimension(); ++d) {
            out[d] = true;
        }
        ParaviewWriter writer(file, position_attr_name, mesh, out[0], out[1], out[2], out[3]);
        mesh.serialize(writer);
    } else if (file.extension() == ".hdf5") {
        output_hdf5(mesh, file);
    } else
        throw std::runtime_error(std::string("Unknown file type: ") + file.string());
}

void output(
    const Mesh& mesh,
    const std::filesystem::path& file,
    const attribute::MeshAttributeHandle& position_attr)
{
    const std::string attr_name = mesh.get_attribute_name(position_attr.as<double>());
    output(mesh, file, attr_name);
}

void output_hdf5(const Mesh& mesh, const std::filesystem::path& file)
{
    HDF5Writer writer(file);
    mesh.serialize(writer);
}


void output(const Mesh& mesh, const OutputOptions& opts)
{
    if (opts.type == ".vtu") {
        assert(opts.position_attribute.index() != std::variant_npos);
        std::string name = std::visit(
            [](const auto& v) -> std::string {
                using T = std::decay_t<decltype(v)>;
                if constexpr (std::is_same_v<T, std::string>) {
                    return v;
                } else if constexpr (std::is_same_v<T, wmtk::attribute::MeshAttributeHandle>) {
                    return v.name();
                }
            },
            opts.position_attribute);
        std::array<bool, 4> out = {{false, false, false, false}};
        for (int64_t d = 0; d <= mesh.top_cell_dimension(); ++d) {
            out[d] = true;
        }
        ParaviewWriter writer(opts.path, name, mesh, out[0], out[1], out[2], out[3]);
        mesh.serialize(writer);
    } else if (opts.type == ".hdf5") {
        output_hdf5(mesh, opts.path);
    } else
        throw std::runtime_error(
            fmt::format("Unable to write file [{}] of extension [{}]", opts.path, opts.type));
}

void output(
    const multimesh::NamedMultiMesh& mesh,
    const OutputOptions& opts,
    const std::string_view& mesh_path)
{
    output(mesh.get_mesh(mesh_path), opts);

    if (opts.mesh_name_path.has_value()) {
        const auto& path = opts.mesh_name_path.value();
        std::ofstream ofs(path);
        ofs << *mesh.get_names_json(mesh_path);
    }
}

void output(
    const multimesh::MeshCollection& mesh_col,
    const std::map<std::string, OutputOptions>& opts)
{
    for (const auto& [path, single_opts] : opts) {
        const multimesh::NamedMultiMesh& nmm = mesh_col.get_named_multimesh(path);
        output(nmm, single_opts, path);
    }
}

} // namespace wmtk::components::output
