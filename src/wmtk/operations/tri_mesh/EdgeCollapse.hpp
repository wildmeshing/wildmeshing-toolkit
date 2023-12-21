
#pragma once

#include "TriMeshOperation.hpp"

namespace wmtk::operations::tri_mesh {
class EdgeCollapse : public TriMeshOperation
{
public:
    // constructor for default factory pattern construction
    EdgeCollapse(Mesh& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

protected:
    std::vector<Simplex> execute(const Simplex& simplex) override;
    std::vector<Simplex> unmodified_primitives(const Simplex& simplex) const override;
};

} // namespace wmtk::operations::tri_mesh
