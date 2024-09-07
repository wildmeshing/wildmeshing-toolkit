#include "AttributeTransferConfiguration.hpp"
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_new/NewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategyBase.hpp>
#include <wmtk/utils/filter_pointers_to_derived.hpp>
namespace wmtk::operations {
namespace internal {


namespace {
template <typename Derived>
auto filter_to_derived(const std::vector<std::shared_ptr<const AttributeTransferEdge>>& edges)
{
    return wmtk::utils::filter_pointers_to_derived<const AttributeTransferEdge, const Derived>(
        edges);
}


} // namespace

class AttributeTransferConfigurationPimpl
{
public:
    void add(const std::shared_ptr<const AttributeTransferEdge>&);


    // This returns a copy because this copy should be passed into some object eventually / i don't intend on keeping the internal data structure as a vector in the long term
    std::vector<std::shared_ptr<const AttributeTransferEdge>> linearized_strategies() const;
    std::vector<std::shared_ptr<const NewAttributeStrategy>> linearized_new_strategies() const;
    std::vector<std::shared_ptr<const AttributeTransferStrategyBase>>
    linearized_transfer_strategies() const;

private:
    std::vector<std::shared_ptr<const AttributeTransferEdge>> m_strats;
};

void AttributeTransferConfigurationPimpl::add(const std::shared_ptr<const AttributeTransferEdge>& s)
{
    m_strats.emplace_back(s);
}
std::vector<std::shared_ptr<const AttributeTransferEdge>>
AttributeTransferConfigurationPimpl::linearized_strategies() const
{
    return m_strats;
}


auto AttributeTransferConfigurationPimpl::linearized_new_strategies() const
    -> std::vector<std::shared_ptr<const NewAttributeStrategy>>
{
    return filter_to_derived<NewAttributeStrategy>(linearized_strategies());
}
auto AttributeTransferConfigurationPimpl::linearized_transfer_strategies() const
    -> std::vector<std::shared_ptr<const AttributeTransferStrategyBase>>
{
    return filter_to_derived<AttributeTransferStrategyBase>(linearized_strategies());
}
} // namespace internal
AttributeTransferConfiguration::AttributeTransferConfiguration()
    : m_impl(std::make_unique<internal::AttributeTransferConfigurationPimpl>())
{}
AttributeTransferConfiguration::~AttributeTransferConfiguration() = default;
void AttributeTransferConfiguration::add(const AttributeTransferEdge& a)
{
    m_impl->add(a.shared_from_this());
}
auto AttributeTransferConfiguration::linearized_strategies() const
    -> std::vector<std::shared_ptr<const AttributeTransferEdge>>
{
    return m_impl->linearized_strategies();
}

auto AttributeTransferConfiguration::linearized_new_strategies() const
    -> std::vector<std::shared_ptr<const NewAttributeStrategy>>
{
    return m_impl->linearized_new_strategies();
}
auto AttributeTransferConfiguration::linearized_transfer_strategies() const
    -> std::vector<std::shared_ptr<const AttributeTransferStrategyBase>>
{
    return m_impl->linearized_transfer_strategies();
}

void AttributeTransferConfiguration::add_split_new(
    const attribute::MeshAttributeHandle& attribute,
    const wmtk::operations::SplitBasicStrategy& spine,
    const wmtk::operations::SplitRibBasicStrategy& rib)
{
    auto ptr = std::visit(
        [&](auto&& val) noexcept -> std::shared_ptr<const AttributeTransferEdge> {
            using HandleType = typename std::decay_t<decltype(val)>;
            if constexpr (attribute::MeshAttributeHandle::template handle_type_is_basic<
                              HandleType>()) {
                using T = typename HandleType::Type;
                using OpType = operations::SplitNewAttributeStrategy<T>;

                std::shared_ptr<OpType> tmp = std::make_shared<OpType>(attribute);
                tmp->set_strategy(spine);
                tmp->set_rib_strategy(rib);

                return tmp;
            } else {
                return {};
            }
        },
        attribute.handle());
    add(*ptr);
}

// convenience for creating collapse starting/new edges
void AttributeTransferConfiguration::add_collapse_new(
    const attribute::MeshAttributeHandle& attribute,
    const wmtk::operations::CollapseBasicStrategy& strategy)
{
    std::visit(
        [&](auto&& val) noexcept -> std::shared_ptr<const AttributeTransferEdge> {
            using HandleType = typename std::decay_t<decltype(val)>;
            if constexpr (attribute::MeshAttributeHandle::template handle_type_is_basic<
                              HandleType>()) {
                using T = typename HandleType::Type;
                using OpType = operations::CollapseNewAttributeStrategy<T>;

                std::shared_ptr<OpType> tmp = std::make_shared<OpType>(attribute);
                tmp->set_strategy(strategy);
                return tmp;
            } else {
                return {};
            }
        },
        attribute.handle());
}

/// @param clear removes all prior attribute transfer behaviors
void AttributeTransferConfiguration::apply(EdgeSplit& split, bool clear) const
{
    auto new_strats =
        internal::filter_to_derived<BaseSplitNewAttributeStrategy>(linearized_strategies());
    for (const auto& s : new_strats) {
        const auto targets = s->targets();
        assert(targets.size() == 1); // unclear if we ever will have more than 1 target.
        split.set_new_attribute_strategy(targets[0], s);
    }

    for (const auto& s : linearized_transfer_strategies()) {
        const auto targets = s->targets();
        assert(targets.size() == 1); // unclear if we ever will have more than 1 target.
        split.set_transfer_strategy(targets[0], s);
    }
    // op.set_new_attribute_strategy(edge_length_handle);
    // op.set_new_attribute_strategy(pos_handle);
    //
}
/// @param clear removes all prior attribute transfer behaviors
void AttributeTransferConfiguration::apply(EdgeCollapse& split, bool clear) const
{
    //
}
} // namespace wmtk::operations
