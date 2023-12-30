#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk::invariants {
/**
 * Invariant for minimum valence on both incident vertices of an edge.
 */
class MinIncidentValenceInvariant : public Invariant
{
public:
    MinIncidentValenceInvariant(const Mesh& m, long min_valence);
    using Invariant::Invariant;
    bool before(const Simplex& t) const override;
    bool after(
        const std::vector<Tuple>& top_dimension_tuples_before,
        const std::vector<Tuple>& top_dimension_tuples_after) const override;

private:
    bool is_greater_min_valence(const Tuple& t) const;

    long m_min_valence;
};
} // namespace wmtk::invariants
