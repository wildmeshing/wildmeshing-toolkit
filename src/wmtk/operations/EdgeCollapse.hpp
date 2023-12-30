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
