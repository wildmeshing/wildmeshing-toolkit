#pragma once

#include "Operation.hpp"

namespace wmtk::operations {

class MeshConsolidate : public Operation
{
public:
    MeshConsolidate(Mesh& m);
    virtual PrimitiveType primitive_type() const override { return PrimitiveType::Vertex; }

protected:
    virtual std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;
    virtual std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

    virtual bool before(const simplex::Simplex& simplex) const override;
    virtual bool after(
        const std::vector<simplex::Simplex>& unmods,
        const std::vector<simplex::Simplex>& mods) const override;
};

} // namespace wmtk::operations
