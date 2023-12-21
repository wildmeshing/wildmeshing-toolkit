#pragma once

#include "TriMeshOperation.hpp"

namespace wmtk::operations::tri_mesh {

class EdgeSplit : public TriMeshOperation
{
public:
    EdgeSplit(Mesh& m);


protected:
    std::vector<Simplex> execute(const Simplex& simplex) override;
    std::vector<Simplex> unmodified_primitives(const Simplex& simplex) const override;

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }
};

} // namespace wmtk::operations::tri_mesh
