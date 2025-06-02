#pragma once

#include "Invariant.hpp"

namespace wmtk::invariants {

/*
 * This invariant keeps substructures in a multimesh separated during collapses.
 *
 * It should be attached to collapse operations alongside the link condition.
 *
 * There are two different options. The default is that all substructures are kept separate. For
 * example, a child A and a child B must remain separate, if they were separate to begin with.
 * However, the invariant can also be restricted to only disallow topological changes in single
 * substructures.
 *
 * The invariant checks two conditions, where the second one can be deactivated if different
 * substructures are allowed to be collapsed into each other. Assume we collapse an edge E and its
 * incident vertices V0 and V1. We denote all substructures a simplex S belongs to by subs(S).
 *
 * Condition 1: The intersection of subs(V0) and subs(V1) is a subset of subs(E).
 * Condition 2 (optional): subs(V0) is a subset of subs(V1) or subs(V1) is a subset of subs(V0).
 *
 */
class SeparateSubstructuresInvariant : public Invariant
{
public:
    SeparateSubstructuresInvariant(const Mesh& m, bool check_condition_2 = true);

    bool before(const simplex::Simplex& s) const override;

private:
    // const bool m_check_condition_2 = true;
};

} // namespace wmtk::invariants