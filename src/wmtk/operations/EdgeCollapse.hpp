#pragma once

#include "MeshOperation.hpp"
#include "attribute_new/CollapseNewAttributeStrategy.hpp"

namespace wmtk::operations {
class EdgeCollapse : public MeshOperation
{
public:
    // constructor for default factory pattern construction
    EdgeCollapse(Mesh& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }


    void set_new_attribute_strategy(
        const attribute::MeshAttributeHandleVariant& attribute,
        const wmtk::operations::CollapseBasicStrategy& strategy =
            wmtk::operations::CollapseBasicStrategy::Default);

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
