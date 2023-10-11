#include "regular_space.hpp"

#include <igl/read_triangle_mesh.h>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>

#include "internal/RegularSpace.hpp"
#include "internal/RegularSpaceOptions.hpp"

namespace wmtk {
namespace components {
void regular_space(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files) {}
} // namespace components
} // namespace wmtk
