#pragma once

#include "Operation.hpp"

namespace wmtk::operations {

class AttributesUpdateBase : public Operation
{
public:
    using Operation::Operation;

    virtual PrimitiveType primitive_type() const override { return PrimitiveType::Vertex; }

protected:
    virtual std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;
    virtual std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;
};

} // namespace wmtk::operations
