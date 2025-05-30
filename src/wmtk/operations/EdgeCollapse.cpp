#include "EdgeCollapse.hpp"
#include <wmtk/utils/Logger.hpp>

#include <cassert>

// #include "utils/multi_mesh_edge_collapse.hpp"

#include <wmtk/submesh/Embedding.hpp>


namespace wmtk::operations {

EdgeCollapse::EdgeCollapse(Mesh& m)
    : Operation(m)
{
    // auto collect_attrs = [&](auto&& mesh) {
    //     // can have const variant values here so gotta filter those out
    //     if constexpr (!std::is_const_v<std::remove_reference_t<decltype(mesh)>>) {
    //         for (const auto& attr : mesh.custom_attributes()) {
    //             std::visit(
    //                 [&](auto&& tah) noexcept {
    //                     using HandleType = typename std::decay_t<decltype(tah)>;
    //                     if constexpr (attribute::MeshAttributeHandle::template
    //                     handle_type_is_basic<
    //                                       HandleType>()) {
    //                         using T = typename HandleType::Type;
    //                         m_new_attr_strategies.emplace_back(
    //                             std::make_shared<operations::CollapseNewAttributeStrategy<T>>(
    //                                 attribute::MeshAttributeHandle(mesh, attr)));
    //                     }
    //                 },
    //                 attr);
    //         }
    //     }
    // };

    // collect_attrs(m);
}

EdgeCollapse::EdgeCollapse(submesh::Embedding& m)
    : EdgeCollapse(m.mesh())
{
    m.set_collapse_strategies(*this);
}

std::vector<simplex::Simplex> EdgeCollapse::execute(const simplex::Simplex& simplex)
{
    throw std::runtime_error("removing multimesh, needs refactoring");
    // return utils::multi_mesh_edge_collapse_with_modified_simplices(
    //     mesh(),
    //     simplex,
    //     m_new_attr_strategies);
}

std::vector<simplex::Simplex> EdgeCollapse::unmodified_primitives(
    const simplex::Simplex& simplex) const
{
    return mesh().parent_scope([&]() -> std::vector<simplex::Simplex> {
        const simplex::Simplex v0 = simplex::Simplex::vertex(mesh(), simplex.tuple());
        const simplex::Simplex v1 = simplex::Simplex::vertex(
            mesh(),
            mesh().switch_tuple(simplex.tuple(), PrimitiveType::Vertex));
        return {v0, v1};
    });
}
////////////////////////////////////

bool EdgeCollapse::after(
    const std::vector<simplex::Simplex>& unmods,
    const std::vector<simplex::Simplex>& mods) const
{
    if (mesh().is_free()) {
        return true;
    } else {
        return Operation::after(unmods, mods);
    }
}
} // namespace wmtk::operations
