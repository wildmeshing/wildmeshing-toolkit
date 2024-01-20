#pragma once
#include "SimplexComparisons.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk::simplex::utils {


inline bool SimplexComparisons::equal(const Mesh& m, const Simplex& s0, const Simplex& s1)
{
    return equal(m, s0.tuple(), s0.primitive_type(), s1.tuple(), s1.primitive_type());
}
inline bool SimplexComparisons::equal(
    const Mesh& m,
    const Tuple& a,
    PrimitiveType a_pt,
    const Tuple& b,
    PrimitiveType b_pt)
{
    return a_pt == b_pt && equal(m, a_pt, a, b);
}
inline bool SimplexComparisons::equal(
    const Mesh& m,
    PrimitiveType primitive_type,
    const Tuple& a,
    const Tuple& b)
{
    return m.id(a, primitive_type) == m.id(b, primitive_type);
}

inline bool SimplexComparisons::less(const Mesh& m, const Simplex& s0, const Simplex& s1)
{
    return less(m, s0.tuple(), s0.primitive_type(), s1.tuple(), s1.primitive_type());
}
inline bool SimplexComparisons::less(
    const Mesh& m,
    const Tuple& a,
    PrimitiveType a_pt,
    const Tuple& b,
    PrimitiveType b_pt)
{
    if (a_pt == b_pt) {
        return less(m, a_pt, a, b);
    } else {
        return a_pt < b_pt;
    }
}
inline bool SimplexComparisons::less(
    const Mesh& m,
    PrimitiveType primitive_type,
    const Tuple& a,
    const Tuple& b)
{
    return m.id(a, primitive_type) < m.id(b, primitive_type);
}
} // namespace wmtk::simplex::utils
