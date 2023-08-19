#pragma once
#include <optional>
#include "Operation.hpp"

namespace wmtk {

class TupleOperation : public Operation
{
public:
    TupleOperation(Mesh& m, const InvariantCollection& invariants, const Tuple& t);
    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    bool before() const override;
    bool after() const override;
    const Tuple& input_tuple() const;

    virtual std::vector<Tuple> modified_primitives(PrimitiveType) const;

private:
    const InvariantCollection& m_invariants;
    Tuple m_input_tuple;
};


} // namespace wmtk
