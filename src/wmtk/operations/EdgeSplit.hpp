#pragma once

#include "MeshOperation.hpp"

namespace wmtk::operations {

class EdgeSplit : public MeshOperation
{
public:
    EdgeSplit(Mesh& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

    static std::pair<Tuple, Tuple> new_spine_edges(const Mesh& mesh, const Tuple& new_vertex);

    void set_standard_strategy(
        const attribute::MeshAttributeHandleVariant& attribute,
        const wmtk::operations::NewAttributeStrategy::SplitBasicStrategy& spine =
            wmtk::operations::NewAttributeStrategy::SplitBasicStrategy::Default,
        const wmtk::operations::NewAttributeStrategy::SplitRibBasicStrategy& rib =
            wmtk::operations::NewAttributeStrategy::SplitRibBasicStrategy::Default);

protected:
    std::vector<simplex::Simplex> execute(EdgeMesh& mesh, const simplex::Simplex& simplex) override;
    std::vector<simplex::Simplex> unmodified_primitives(
        const EdgeMesh& mesh,
        const simplex::Simplex& simplex) const override;

    std::vector<simplex::Simplex> execute(TriMesh& mesh, const simplex::Simplex& simplex) override;
    std::vector<simplex::Simplex> unmodified_primitives(
        const TriMesh& mesh,
        const simplex::Simplex& simplex) const override;

    std::vector<simplex::Simplex> execute(TetMesh& mesh, const simplex::Simplex& simplex) override;
    std::vector<simplex::Simplex> unmodified_primitives(
        const TetMesh& mesh,
        const simplex::Simplex& simplex) const override;
};

} // namespace wmtk::operations
