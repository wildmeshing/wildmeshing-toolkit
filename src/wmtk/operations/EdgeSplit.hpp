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
    std::vector<Simplex> execute(EdgeMesh& mesh, const Simplex& simplex) override;
    std::vector<Simplex> unmodified_primitives(const EdgeMesh& mesh, const Simplex& simplex)
        const override;

    std::vector<Simplex> execute(TriMesh& mesh, const Simplex& simplex) override;
    std::vector<Simplex> unmodified_primitives(const TriMesh& mesh, const Simplex& simplex)
        const override;

    std::vector<Simplex> execute(TetMesh& mesh, const Simplex& simplex) override;
    std::vector<Simplex> unmodified_primitives(const TetMesh& mesh, const Simplex& simplex)
        const override;
};

} // namespace wmtk::operations
