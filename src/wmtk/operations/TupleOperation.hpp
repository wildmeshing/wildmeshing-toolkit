#pragma once
#include <optional>
#include "Operation.hpp"

namespace wmtk::operations {

class TupleOperation : virtual public Operation
{
public:
    TupleOperation(const InvariantCollection& invariants, const Tuple& t);
    // especially in the case of compound operations we might not know the input tuple at
    // construction we therefore have to pass in a default invalid tuple and set the tuple later on
    TupleOperation(const InvariantCollection& invariants);
    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    bool before() const override;
    bool after() const override;
    const Tuple& input_tuple() const;

    //
    void set_input_tuple(const Tuple& t);

    // Returns the set of tuples, organized by the type
    virtual std::vector<Tuple> modified_primitives(PrimitiveType) const;

private:
    const InvariantCollection& m_invariants;
    Tuple m_input_tuple;
};


} // namespace wmtk::operations
