#include "from_boundary.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include "from_tag.hpp"


namespace wmtk::components::multimesh {


void from_boundary(
    Mesh& mesh,
    const PrimitiveType ptype,
    const std::string& attribute_name,
    char value,
    std::vector<wmtk::attribute::MeshAttributeHandle>& passed_attributes)
{
    auto is_boundary_handle = mesh.register_attribute<char>(attribute_name, ptype, 1);
    auto is_boundary_accessor = mesh.create_accessor(is_boundary_handle.as<char>());

    for (const auto& t : mesh.get_all(ptype)) {
        is_boundary_accessor.scalar_attribute(t) = mesh.is_boundary(ptype, t) ? value : 0;
    }

    from_tag(is_boundary_handle, value, passed_attributes);
}

} // namespace wmtk::components::multimesh
