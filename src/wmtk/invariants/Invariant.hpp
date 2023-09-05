#pragma once
#include <vector>
#include <wmtk/Primitive.hpp>

namespace wmtk {
class Tuple;
class Invariant
{
public:
    // invariants can add constraints on either the before or after of a function
    // The default implementation is that both constraints are true so derived classes only have to
    // define one of the two
    virtual bool before(const Tuple& t) const;
    virtual bool after(PrimitiveType type, const std::vector<Tuple>& t) const;
    virtual ~Invariant();
};

} // namespace wmtk
