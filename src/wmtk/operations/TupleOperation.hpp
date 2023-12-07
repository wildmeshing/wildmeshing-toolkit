#pragma once
#include <optional>
#include <wmtk/Simplex.hpp>
#include "Operation.hpp"

namespace wmtk::operations {

class TupleOperation : virtual public Operation
{
public:
    TupleOperation(std::shared_ptr<InvariantCollection> invariants, const Simplex& t);
    // especially in the case of compound operations we might not know the input tuple at
    // construction we therefore have to pass in a default invalid tuple and set the tuple later on
    // TupleOperation(std::shared_ptr<InvariantCollection> invariants);
    // TODO what is this now?
    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    bool before() const override;
    bool after() const override;
    const Simplex& input_tuple() const; // TODO rename to input_simplex

    //
    void set_input_tuple(const Simplex& t); // TODO rename to set_input_simplex

    // Returns the set of tuples, organized by the type
    virtual std::vector<Tuple> modified_primitives(PrimitiveType) const;


    const InvariantCollection& invariants() const { return *m_invariants; }
    std::shared_ptr<InvariantCollection> invariants_pointer() const { return m_invariants; }

private:
    std::shared_ptr<InvariantCollection> m_invariants;
    Simplex m_input_tuple;
};


} // namespace wmtk::operations
