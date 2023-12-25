#include "paraview_write_child_meshes.hpp"
#include <wmtk/Accessor.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
namespace wmtk::io::utils {
void write_child_meshes(
    const std::string& filename,
    Mesh* child_mesh,
    const Mesh* parent_mesh,
    const MeshAttributeHandle<double>& parent_handle)
{
    wmtk::io::ParaviewWriter writer;
    switch (child_mesh->top_cell_dimension()) {
    case 0:
        writer.init(filename, "child_vertices", *child_mesh, {{true, false, false, false}});
        break;
    case 1:
        writer.init(filename, "child_vertices", *child_mesh, {{true, true, false, false}});
        break;
    case 2:
        writer.init(filename, "child_vertices", *child_mesh, {{true, true, true, false}});
        break;
    case 3: writer.init(filename, "child_vertices", *child_mesh, {{true, true, true, true}}); break;
    default: throw std::runtime_error("Unsupported dimension!");
    }

    long dim = parent_mesh->get_attribute_dimension<double>(parent_handle);
    MeshAttributeHandle<double> child_vertex_handle =
        child_mesh->register_attribute<double>("child_vertices", PrimitiveType::Vertex, dim);
    Accessor<double> child_vertex_accessor =
        child_mesh->create_accessor<double>(child_vertex_handle);
    ConstAccessor<double> parent_vertex_accessor =
        parent_mesh->create_const_accessor<double>(parent_handle);

    for (Tuple& v : child_mesh->get_all(PrimitiveType::Vertex)) {
        Tuple parent_v = child_mesh->map_to_parent_tuple(Simplex(PrimitiveType::Vertex, v));
        child_vertex_accessor.vector_attribute(v) =
            parent_vertex_accessor.const_vector_attribute(parent_v);
    }
    child_mesh->serialize(writer);
}
} // namespace wmtk::io::utils