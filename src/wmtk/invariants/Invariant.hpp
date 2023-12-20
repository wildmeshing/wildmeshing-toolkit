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
    virtual bool after(
        const std::vector<Tuple>& top_dimension_tuples_before,
        const std::vector<Tuple>& top_dimension_tuples_after) const;
    Invariant(const Mesh& m);
    virtual ~Invariant();

    const Mesh& mesh() const;

    // TODO change name
    virtual bool directly_modified_after(
        const std::vector<simplex::Simplex>& simplices_before,
        const std::vector<simplex::Simplex>& simplices_after) const;

private:
    const Mesh& m_mesh;

    const std::vector<Tuple> get_top_dimension_cofaces(
        const std::vector<simplex::Simplex>& simplices) const;
};
} // namespace invariants
using Invariant = invariants::Invariant;

} // namespace wmtk
