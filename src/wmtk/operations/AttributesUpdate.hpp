#pragma once

#include "Operation.hpp"

namespace wmtk::operations {

class AttributesUpdate : public Operation
{
public:
    AttributesUpdate(Mesh& m);

    virtual PrimitiveType primitive_type() const override { return PrimitiveType::Vertex; }

protected:
    virtual std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override;
    virtual std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;
};

class AttributesUpdateWithFunction : public AttributesUpdate
{
public:
    AttributesUpdateWithFunction(Mesh& m);

    using UpdateFunction = std::function<void(Mesh&, const simplex::Simplex& s)>;

    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;

    void set_function(const UpdateFunction& func) { m_function = func; }

private:
    UpdateFunction m_function;
};

} // namespace wmtk::operations
