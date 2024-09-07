#pragma once

#include <memory>
#include <vector>
#include <wmtk/operations/attribute_new/Enums.hpp>
namespace wmtk {

namespace attribute {
class MeshAttributeHandle;
}
namespace operations {
namespace internal {
class AttributeTransferConfigurationPimpl;
}
class BaseSplitAttributeStrategy;
class NewAttributeStrategy;
class AttributeTransferEdge;
class AttributeTransferStrategyBase;

class EdgeCollapse;
class EdgeSplit;

class AttributeTransferConfiguration
{
public:
    AttributeTransferConfiguration();
    ~AttributeTransferConfiguration();

    void add(const AttributeTransferEdge&);


    // convenience for creating split starting/new edges
    void add_split_new(
        const attribute::MeshAttributeHandle& attribute,
        const wmtk::operations::SplitBasicStrategy& spine =
            wmtk::operations::SplitBasicStrategy::Default,
        const wmtk::operations::SplitRibBasicStrategy& rib =
            wmtk::operations::SplitRibBasicStrategy::Default);

    // convenience for creating collapse starting/new edges
    void add_collapse_new(
        const attribute::MeshAttributeHandle& attribute,
        const wmtk::operations::CollapseBasicStrategy& strategy =
            wmtk::operations::CollapseBasicStrategy::Default);


    std::vector<std::shared_ptr<const AttributeTransferEdge>> linearized_strategies() const;
    std::vector<std::shared_ptr<const NewAttributeStrategy>> linearized_new_strategies() const;
    std::vector<std::shared_ptr<const AttributeTransferStrategyBase>>
    linearized_transfer_strategies() const;


    /// @param clear removes all prior attribute transfer behaviors
    void apply(EdgeSplit& split, bool clear = false) const;
    /// @param clear removes all prior attribute transfer behaviors
    void apply(EdgeCollapse& split, bool clear = false) const;

private:
    std::unique_ptr<internal::AttributeTransferConfigurationPimpl> m_impl;
};
} // namespace operations
} // namespace wmtk
