#pragma once

#include "MeshOperation.hpp"

namespace wmtk::operations {
class EdgeCollapse : public MeshOperation
{
public:
    // constructor for default factory pattern construction
    EdgeCollapse(Mesh& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }


    void set_standard_strategy(
        const attribute::MeshAttributeHandleVariant& attribute,
        const wmtk::operations::NewAttributeStrategy::CollapseBasicStrategy& strategy =
            wmtk::operations::NewAttributeStrategy::CollapseBasicStrategy::Default);

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
