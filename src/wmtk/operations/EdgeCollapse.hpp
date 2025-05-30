#pragma once

#include "Operation.hpp"

namespace wmtk::submesh {
class Embedding;
}

namespace wmtk::operations {
class EdgeCollapse : public Operation
{
public:
    // constructor for default factory pattern construction
    EdgeCollapse(Mesh& m);
    EdgeCollapse(submesh::Embedding& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;

    // for free meshes after is a no-op because the simplex is just gone
    bool after(
        const std::vector<simplex::Simplex>& unmods,
        const std::vector<simplex::Simplex>& mods) const final override;

private:
};

} // namespace wmtk::operations
