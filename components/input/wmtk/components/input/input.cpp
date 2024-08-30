#include "input.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <jse/jse.h>

#include "internal/InputOptions.hpp"

#include "input_spec.hpp"

namespace wmtk::components {

std::shared_ptr<Mesh> input(nlohmann::json j)
{
    using namespace internal;

    jse::JSE spec_engine;
    spec_engine.strict = true;

    bool r = spec_engine.verify_json(j, input_spec);
    if (!r) {
        log_and_throw_error("{}", spec_engine.log2str());
    } else {
        j = spec_engine.inject_defaults(j, input_spec);
    }

    const InputOptions options = j.get<InputOptions>();

    return input(options.file, options.ignore_z, options.tetrahedron_attributes);
}

std::shared_ptr<Mesh>
input(const std::filesystem::path& file, const bool ignore_z, const std::vector<std::string> tetrahedron_attributes)
{
    if (!std::filesystem::exists(file)) {
        log_and_throw_error("file {} not found", file);
    }

    const std::shared_ptr<Mesh> mesh = read_mesh(file, ignore_z, tetrahedron_attributes);
    assert(mesh->is_connectivity_valid());

    return mesh;
}
} // namespace wmtk::components