#pragma once

#include "MeshOperation.hpp"

#include "attribute_new/SplitNewAttributeStrategy.hpp"

namespace wmtk::operations {

class EdgeSplit : public MeshOperation
{
public:
    EdgeSplit(Mesh& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

    static std::pair<Tuple, Tuple> new_spine_edges(const Mesh& mesh, const Tuple& new_vertex);


    std::shared_ptr<operations::BaseSplitNewAttributeStrategy> get_new_attribute_strategy(
        const attribute::MeshAttributeHandle& attribute) const;

    void set_new_attribute_strategy(
        const attribute::MeshAttributeHandle& attribute,
        const std::shared_ptr<operations::BaseSplitNewAttributeStrategy>& other);

    void set_new_attribute_strategy(
        const attribute::MeshAttributeHandle& attribute,
        const wmtk::operations::SplitBasicStrategy& spine =
            wmtk::operations::SplitBasicStrategy::Default,
        const wmtk::operations::SplitRibBasicStrategy& rib =
            wmtk::operations::SplitRibBasicStrategy::Default);

protected:
    std::vector<simplex::Simplex> execute_aux(EdgeMesh& mesh, const simplex::Simplex& simplex)
        override;
    std::vector<simplex::Simplex> unmodified_primitives_aux(
        const EdgeMesh& mesh,
        const simplex::Simplex& simplex) const override;

    std::vector<simplex::Simplex> execute_aux(TriMesh& mesh, const simplex::Simplex& simplex)
        override;
    std::vector<simplex::Simplex> unmodified_primitives_aux(
        const TriMesh& mesh,
        const simplex::Simplex& simplex) const override;

    std::vector<simplex::Simplex> execute_aux(TetMesh& mesh, const simplex::Simplex& simplex)
        override;
    std::vector<simplex::Simplex> unmodified_primitives_aux(
        const TetMesh& mesh,
        const simplex::Simplex& simplex) const override;

private:
    std::vector<std::shared_ptr<operations::BaseSplitNewAttributeStrategy>> m_new_attr_strategies;
};

} // namespace wmtk::operations
