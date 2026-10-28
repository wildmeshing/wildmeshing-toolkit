#include "read_mesh.hpp"

#include <stdexcept>

namespace wmtk::components::prismatic_mesh {
PrismaticMeshInput load_prismatic_mesh(const std::filesystem::path& path)
{
    if (path.extension() == ".vtu") return detail::load_prismatic_vtu(path);
    if (path.extension() == ".msh") return detail::load_prismatic_msh(path);
    throw std::runtime_error("prismatic_mesh: input must be a .vtu or .msh file");
}
} // namespace wmtk::components::prismatic_mesh
