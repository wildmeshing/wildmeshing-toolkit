#pragma once

#include "Operation.hpp"

namespace wmtk::operations {

class MeshOperation : public Operation
{
public:
    MeshOperation(Mesh& m);
    ~MeshOperation() = default;

protected:
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;

    virtual std::vector<simplex::Simplex> execute_aux(
        EdgeMesh& mesh,
        const simplex::Simplex& simplex) = 0;
    virtual std::vector<simplex::Simplex> unmodified_primitives_aux(
        const EdgeMesh& mesh,
        const simplex::Simplex& simplex) const = 0;

    virtual std::vector<simplex::Simplex> execute_aux(
        TriMesh& mesh,
        const simplex::Simplex& simplex) = 0;
    virtual std::vector<simplex::Simplex> unmodified_primitives_aux(
        const TriMesh& mesh,
        const simplex::Simplex& simplex) const = 0;

    virtual std::vector<simplex::Simplex> execute_aux(
        TetMesh& mesh,
        const simplex::Simplex& simplex) = 0;
    virtual std::vector<simplex::Simplex> unmodified_primitives_aux(
        const TetMesh& mesh,
        const simplex::Simplex& simplex) const = 0;
};
} // namespace wmtk::operations
