#pragma once

#include "Operation.hpp"

namespace wmtk::operations {

class AttributesUpdateBase : public Operation
{
public:
    AttributesUpdateBase(Mesh& m);

    virtual PrimitiveType primitive_type() const override { return PrimitiveType::Vertex; }

protected:
    virtual std::vector<Simplex> unmodified_primitives(const Simplex& simplex) const override;
    virtual std::vector<Simplex> execute(const Simplex& simplex) override;
};

} // namespace wmtk::operations
