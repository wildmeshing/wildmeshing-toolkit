#pragma once

#include "Operation.hpp"

namespace wmtk::operations {

class MeshOperation : public Operation
{
public:
    MeshOperation(Mesh& m);
    ~MeshOperation() = default;

protected:
    std::vector<Simplex> execute(const Simplex& simplex) override;
    std::vector<Simplex> unmodified_primitives(const Simplex& simplex) const override;

    virtual std::vector<Simplex> execute(EdgeMesh& mesh, const Simplex& simplex) = 0;
    virtual std::vector<Simplex> unmodified_primitives(const EdgeMesh& mesh, const Simplex& simplex)
        const = 0;

    virtual std::vector<Simplex> execute(TriMesh& mesh, const Simplex& simplex) = 0;
    virtual std::vector<Simplex> unmodified_primitives(const TriMesh& mesh, const Simplex& simplex)
        const = 0;

    virtual std::vector<Simplex> execute(TetMesh& mesh, const Simplex& simplex) = 0;
    virtual std::vector<Simplex> unmodified_primitives(const TetMesh& mesh, const Simplex& simplex)
        const = 0;
};
} // namespace wmtk::operations
