#include "EdgeCollapse.hpp"

#include <cassert>
#include <wmtk/operations/utils/multi_mesh_edge_collapse.hpp>
#include "attribute_new/CollapseNewAttributeStrategy.hpp"

#if defined(WMTK_ENABLE_MULTIMESH)
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include "utils/multi_mesh_edge_collapse.hpp"
#else
#include <wmtk/utils/metaprogramming/as_mesh_variant.hpp>
#include "utils/EdgeCollapseFunctor.hpp"
#endif


namespace wmtk::operations {


EdgeCollapse::EdgeCollapse(Mesh& m)
    : Operation(m)
{
    auto collect_attrs = [&](auto&& mesh) {
        // can have const variant values here so gotta filter htose out
        if constexpr (!std::is_const_v<std::remove_reference_t<decltype(mesh)>>) {
            for (const auto& attr : mesh.custom_attributes()) {
                std::visit(
                    [&](auto&& tah) noexcept {
                        using HandleType = typename std::decay_t<decltype(tah)>;
                        if constexpr (attribute::MeshAttributeHandle::template handle_type_is_basic<
                                          HandleType>()) {
                            using T = typename HandleType::Type;
                            m_new_attr_strategies.emplace_back(
                                std::make_shared<operations::CollapseNewAttributeStrategy<T>>(
                                    attribute::MeshAttributeHandle(mesh, attr)));
                        }
                    },
                    attr);
            }
        }
    };

#if defined(WMTK_ENABLE_MULTIMESH)
    multimesh::MultiMeshVisitor custom_attribute_collector(collect_attrs);
    custom_attribute_collector.execute_from_root(m);
#else
    collect_attrs(m);
#endif
}

std::vector<simplex::Simplex> EdgeCollapse::execute(const simplex::Simplex& simplex)
{
#if defined(WMTK_ENABLE_MULTIMESH)

    return utils::multi_mesh_edge_collapse_with_modified_simplices(
        mesh(),
        simplex,
        m_new_attr_strategies);
#else
    return std::visit([&](auto&& mesh_ref) noexcept -> std::vector<simplex::Simplex> {

            using T = std::decay_t<typename std::decay_t<decltype(mesh_ref)>::type>;
            if constexpr(!(std::is_same_v<Mesh,T> || std::is_same_v<PointMesh,T>)) {
           auto edge_op_data = utils::EdgeCollapseFunctor{}(mesh_ref.get(), simplex);

           return {simplex::Simplex::vertex(edge_op_data.m_output_tuple)};
           } else {
           return {};
           }
           }, wmtk::utils::metaprogramming::as_mesh_variant(mesh()));

#endif
}

std::vector<simplex::Simplex> EdgeCollapse::unmodified_primitives(
    const simplex::Simplex& simplex) const
{
    return mesh().parent_scope([&]() -> std::vector<simplex::Simplex> {
        const simplex::Simplex v0 = simplex::Simplex::vertex(simplex.tuple());
        const simplex::Simplex v1 =
            simplex::Simplex::vertex(mesh().switch_tuple(simplex.tuple(), PrimitiveType::Vertex));
        return {v0, v1};
    });
}
////////////////////////////////////


std::shared_ptr<operations::BaseCollapseNewAttributeStrategy>
EdgeCollapse::get_new_attribute_strategy(const attribute::MeshAttributeHandle& attribute) const
{
    for (auto& s : m_new_attr_strategies) {
        if (s->matches_attribute(attribute)) return s;
    }

    throw std::runtime_error("unable to find attribute");
}

void EdgeCollapse::set_new_attribute_strategy(
    const attribute::MeshAttributeHandle& attribute,
    const std::shared_ptr<operations::BaseCollapseNewAttributeStrategy>& other)
{
    for (size_t i = 0; i < m_new_attr_strategies.size(); ++i) {
        if (m_new_attr_strategies[i]->matches_attribute(attribute)) {
            m_new_attr_strategies[i] = other;
            return;
        }
    }

    throw std::runtime_error("unable to find attribute");
}

void EdgeCollapse::set_new_attribute_strategy(
    const attribute::MeshAttributeHandle& attribute,
    const wmtk::operations::CollapseBasicStrategy& strategy)
{
    std::visit(
        [&](auto&& val) noexcept -> void {
            using HandleType = typename std::decay_t<decltype(val)>;
            if constexpr (attribute::MeshAttributeHandle::template handle_type_is_basic<
                              HandleType>()) {
                using T = typename HandleType::Type;
                using OpType = operations::CollapseNewAttributeStrategy<T>;

                std::shared_ptr<OpType> tmp = std::make_shared<OpType>(attribute);
                tmp->set_strategy(strategy);

                set_new_attribute_strategy(attribute, tmp);
            }
        },
        attribute.handle());
}

} // namespace wmtk::operations
