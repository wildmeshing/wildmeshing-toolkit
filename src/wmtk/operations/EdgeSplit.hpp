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
};

} // namespace wmtk::operations
