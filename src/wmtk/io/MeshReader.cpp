#include "MeshReader.hpp"
#include "read_mesh.hpp"


#include <wmtk/utils/Logger.hpp>

#include <memory>

namespace wmtk {

std::shared_ptr<Mesh> read_mesh(
    const std::filesystem::path& filename,
    const bool ignore_z_if_zero,
    const std::vector<std::string>& tetrahedron_attributes)
{
    return io::read_mesh(filename, ignore_z_if_zero, tetrahedron_attributes);
}

} // namespace wmtk
