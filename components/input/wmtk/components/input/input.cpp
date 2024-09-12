#include "input.hpp"

#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::components {

std::shared_ptr<Mesh> input(
    const std::filesystem::path& file,
    const bool ignore_z,
    const std::vector<std::string>& tetrahedron_attributes)
{
    if (!std::filesystem::exists(file)) {
        log_and_throw_error("file {} not found", std::string(file));
    }

    const std::shared_ptr<Mesh> mesh = read_mesh(file, ignore_z, tetrahedron_attributes);
    assert(mesh->is_connectivity_valid());

    return mesh;
}
} // namespace wmtk::components
