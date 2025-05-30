#pragma once

#include "Operation.hpp"

namespace wmtk::submesh {
class Embedding;
}

namespace wmtk::operations {
class EdgeSplit : public Operation
{
public:
    EdgeSplit(Mesh& m);
    EdgeSplit(submesh::Embedding& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Edge; }

    static std::pair<Tuple, Tuple> new_spine_edges(const Mesh& mesh, const Tuple& new_vertex);

    /**
     * @return the new simplex, toward to the next simplex along the splited line
     */
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;


private:
};

} // namespace wmtk::operations
