#include "MeshReader.hpp"
#include "read_mesh.hpp"


#include <wmtk/utils/Logger.hpp>

#include <memory>

namespace wmtk {

std::shared_ptr<Mesh> read_mesh(
    const std::filesystem::path& filename,
    const bool ignore_z,
    const std::vector<std::string>& tetrahedron_attributes)
{
    return io::read_mesh(filename, ignore_z, tetrahedron_attributes);
}

} // namespace wmtk
