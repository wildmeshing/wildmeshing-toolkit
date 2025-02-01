#pragma once

#include "Operation.hpp"
#include "attribute_new/CollapseNewAttributeStrategy.hpp"

namespace wmtk::operations {
class EdgeCollapse : public Operation
{
public:
    // constructor for default factory pattern construction
    EdgeCollapse(Mesh& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }


    std::shared_ptr<const operations::BaseCollapseNewAttributeStrategy> get_new_attribute_strategy(
        const attribute::MeshAttributeHandle& attribute) const;

    void set_new_attribute_strategy(
        const attribute::MeshAttributeHandle& attribute,
        const std::shared_ptr<const operations::BaseCollapseNewAttributeStrategy>& other);

    void clear_attribute_new_strategies();

    void set_new_attribute_strategy(
        const attribute::MeshAttributeHandle& attribute,
        const wmtk::operations::CollapseBasicStrategy& strategy =
            wmtk::operations::CollapseBasicStrategy::Default);

    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;

    // for free meshes after is a no-op because the simplex is just gone
    bool after(
        const std::vector<simplex::Simplex>& unmods,
        const std::vector<simplex::Simplex>& mods) const final override;

    // checks through attribute new for throws, prints to the logger if so
    bool attribute_new_all_configured() const;

private:
    std::vector<std::shared_ptr<const operations::BaseCollapseNewAttributeStrategy>>
        m_new_attr_strategies;
};

} // namespace wmtk::operations
