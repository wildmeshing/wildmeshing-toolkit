#include "add_free_child_mesh.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>


namespace wmtk::tests {
std::shared_ptr<Mesh> add_free_child_mesh(Mesh& m, PrimitiveType pt)
{
    auto attr_handle = m.register_attribute_typed<int64_t>("child_tag", pt, 1, true, int64_t(0));

    auto cm_ptr = wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag_handle(
        m,
        attr_handle,
        0,
        true);

    return cm_ptr;
}
} // namespace wmtk::tests
