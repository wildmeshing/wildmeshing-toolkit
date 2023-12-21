#pragma once
#include <optional>
#include <wmtk/Simplex.hpp>
#include "Operation.hpp"

namespace wmtk::operations {

class TupleOperation : virtual public Operation
{
public:
    friend class OperationQueue;
    TupleOperation(std::shared_ptr<InvariantCollection> invariants, const Simplex& t);
    // especially in the case of compound operations we might not know the input tuple at
    // construction we therefore have to pass in a default invalid tuple and set the tuple later on
    // TupleOperation(std::shared_ptr<InvariantCollection> invariants);
    // TODO what is this now?
    // static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    bool before() const override;
    bool after() const override;
    const Tuple& input_tuple() const;
    const Simplex& input_simplex() const;

    //
    void set_input_simplex(const Simplex& t);

    /**
     * Returns all simplices that are potentially affected by the operation.
     */
    virtual std::vector<Simplex> modified_primitives() const = 0;

    /**
     * Returns all simplices that will be potentially affected by the operation
     */
    virtual std::vector<Simplex> unmodified_primitives() const = 0;


    const InvariantCollection& invariants() const { return *m_invariants; }
    std::shared_ptr<InvariantCollection> invariants_pointer() const { return m_invariants; }


    std::array<Tuple, 2> new_spine_edges() const;

private:
    std::shared_ptr<InvariantCollection> m_invariants;
    Simplex m_input_simplex;
};


} // namespace wmtk::operations
