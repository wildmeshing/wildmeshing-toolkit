#pragma once
#include <array>
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

    // have an invariant that sets all three by default
    Invariant(const Mesh& m);
    Invariant(
        const Mesh& m,
        bool use_before,
        bool use_old_state_in_after,
        bool use_new_state_in_after);
    virtual ~Invariant();

    const Mesh& mesh() const;

    // A compact pipeline for evaluating after without computing any cofaces
    // TODO change name
    virtual bool directly_modified_after(
        const std::vector<simplex::Simplex>& simplices_before,
        const std::vector<simplex::Simplex>& simplices_after) const;


    bool use_before() const;
    bool use_after() const;
    bool use_old_state_in_after() const;
    bool use_new_state_in_after() const;

    virtual bool is_collection() const;

private:
    const Mesh& m_mesh;
    const bool m_use_before = true;
    const bool m_use_old_state_in_after = true;
    const bool m_use_new_state_in_after = true;

protected:
    const std::vector<Tuple> get_top_dimension_cofaces(
        const std::vector<simplex::Simplex>& simplices) const;
};
} // namespace invariants
using Invariant = invariants::Invariant;

} // namespace wmtk
