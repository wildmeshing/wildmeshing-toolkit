#pragma once
#include <vector>
#include <wmtk/PrimitiveType.hpp>
namespace wmtk {
class Tuple;
class Mesh;
namespace simplex {
class Simplex;
}
namespace invariants {
class Invariant
{
public:
    // invariants can add constraints on either the before or after of a function
    // The default implementation is that both constraints are true so derived classes only have to
    // define one of the two
    virtual bool before(const simplex::Simplex& t) const;
    virtual bool after(PrimitiveType type, const std::vector<Tuple>& t) const;
    Invariant(const Mesh& m);
    virtual ~Invariant();

    const Mesh& mesh() const;

    virtual bool directly_modified_after(PrimitiveType type, const std::vector<Tuple>& t) const;

private:
    const Mesh& m_mesh;
};
} // namespace invariants
using Invariant = invariants::Invariant;

} // namespace wmtk
