#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/simplex/Simplex.hpp>
using namespace wmtk;
using namespace wmtk::attribute;
namespace wmtk::io::utils {
void write_child_meshes(
    const std::string& filename,
    Mesh* child_mesh,
    const Mesh* parent_mesh,
    const MeshAttributeHandle& parent_handle);
} // namespace wmtk::io::utils