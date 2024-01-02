#include <wmtk/Accessor.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/simplex/Simplex.hpp>
namespace wmtk::io::utils {
void write_child_meshes(
    const std::string& filename,
    Mesh* child_mesh,
    const Mesh* parent_mesh,
    const MeshAttributeHandle<double>& parent_handle);
} // namespace wmtk::io::utils