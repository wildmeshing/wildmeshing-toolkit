#include "EdgeCollapse.hpp"
#include <wmtk/utils/Logger.hpp>

#include <cassert>
#include <wmtk/operations/utils/multi_mesh_edge_collapse.hpp>
#include "attribute_new/CollapseNewAttributeStrategy.hpp"

#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#include "utils/multi_mesh_edge_collapse.hpp"


namespace wmtk::operations {


EdgeCollapse::EdgeCollapse(Mesh& m)
    : Operation(m)
{
    operation_name = "EdgeCollapse";
    auto collect_attrs = [&](auto&& mesh) {
        // can have const variant values here so gotta filter htose out
        if constexpr (!std::is_const_v<std::remove_reference_t<decltype(mesh)>>) {
            if (mesh.is_free()) {
                return;
            }
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

    multimesh::MultiMeshVisitor custom_attribute_collector(collect_attrs);
    custom_attribute_collector.execute_from_root(m);
}

std::vector<simplex::Simplex> EdgeCollapse::execute(const simplex::Simplex& simplex)
{
    return utils::multi_mesh_edge_collapse_with_modified_simplices(
        mesh(),
        simplex,
        m_new_attr_strategies);
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
    if(attribute.mesh().is_free()) {
        logger().debug("Set new collapse attribute strategy on a free mesh, there are no new attributes for free mesh collapses");
        return;
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
