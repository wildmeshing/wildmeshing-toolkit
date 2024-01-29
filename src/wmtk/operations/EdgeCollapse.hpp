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


    std::shared_ptr<operations::BaseCollapseNewAttributeStrategy> get_new_attribute_strategy(
        const attribute::MeshAttributeHandle& attribute) const;

    void set_new_attribute_strategy(
        const attribute::MeshAttributeHandle& attribute,
        const std::shared_ptr<operations::BaseCollapseNewAttributeStrategy>& other);


    void set_new_attribute_strategy(
        const attribute::MeshAttributeHandle& attribute,
        const wmtk::operations::CollapseBasicStrategy& strategy =
            wmtk::operations::CollapseBasicStrategy::Default);

    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;

private:
    std::vector<std::shared_ptr<operations::BaseCollapseNewAttributeStrategy>>
        m_new_attr_strategies;

    template <typename MeshType>
    std::vector<simplex::Simplex> execute_tmpl(MeshType& mesh, const simplex::Simplex& simplex);
    template <typename MeshType>
    std::vector<simplex::Simplex> unmodified_primitives_tmpl(
        const MeshType& mesh,
        const simplex::Simplex& simplex) const;
};

} // namespace wmtk::operations
