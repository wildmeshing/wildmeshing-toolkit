#pragma once
#include <wmtk/Mesh.hpp>
#include "SimplexComparisons.hpp"

namespace wmtk::simplex::utils {


inline bool SimplexComparisons::equal(const Mesh& m, const Simplex& s0, const Simplex& s1)
{
#if defined(WMTK_ENABLE_SIMPLEX_ID_CACHING)
    return s0 == s1;
#else
    const auto s0pt = s0.primitive_type();
    const auto s1pt = s1.primitive_type();
    const auto s0id = m.id(s0);
    const auto s1id = m.id(s1);
    return std::tie(s0pt, s0id) == std::tie(s1pt, s1id);
#endif
    //    return equal(m, s0.tuple(), s0.primitive_type(), s1.tuple(), s1.primitive_type());
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
    return a == b || m.id(a, primitive_type) == m.id(b, primitive_type);
}

inline bool SimplexComparisons::less(const Mesh& m, const Simplex& s0, const Simplex& s1)
{
#if defined(WMTK_ENABLE_SIMPLEX_ID_CACHING)
    return s0 < s1;
#else
    const auto s0pt = s0.primitive_type();
    const auto s1pt = s1.primitive_type();
    const auto s0id = m.id(s0);
    const auto s1id = m.id(s1);
    return std::tie(s0pt, s0id) < std::tie(s1pt, s1id);
#endif
    //    return less(m, s0.tuple(), s0.primitive_type(), s1.tuple(), s1.primitive_type());
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
    if (a == b) {
        return false;
    }
    return m.id(a, primitive_type) < m.id(b, primitive_type);
}
inline bool SimplexComparisons::equal_subdart(
    const Mesh& m,
    PrimitiveType primitive_type,
    const Tuple& a,
    const Tuple& b)
{
    for (PrimitiveType pt = PrimitiveType::Vertex; pt < primitive_type; pt = pt + 1) {
        if (m.id(a, pt) != m.id(b, pt)) {
            return false;
        }
    }
    return true;
}

inline bool SimplexComparisons::equal_subdart(
    const Mesh& m,
    PrimitiveType primitive_type,
    const Simplex& a,
    const Simplex& b)
{
    return equal_subdart(m, primitive_type, a.tuple(), b.tuple());
}
inline bool SimplexComparisons::equal_subdart(const Mesh& m, const Simplex& a, const Simplex& b)
{
    PrimitiveType primitive_type =
        a.primitive_type() < b.primitive_type() ? a.primitive_type() : b.primitive_type();
    return equal_subdart(m, primitive_type, a, b);
}

bool SimplexComparisons::less_subdart(
    const Mesh& m,
    PrimitiveType primitive_type,
    const Tuple& a,
    const Tuple& b)
{
    for (PrimitiveType pt = PrimitiveType::Vertex; pt < primitive_type; pt = pt + 1) {
        int64_t ai = m.id(a, pt);
        int64_t bi = m.id(b, pt);
        if (ai == bi) {
            continue;
        } else {
            return ai < bi;
        }
    }
    return false;
}
bool SimplexComparisons::less_subdart(
    const Mesh& m,
    PrimitiveType primitive_type,
    const Simplex& a,
    const Simplex& b)
{
    return less_subdart(m, primitive_type, a.tuple(), b.tuple());
}
bool SimplexComparisons::less_subdart(const Mesh& m, const Simplex& a, const Simplex& b)
{
    PrimitiveType primitive_type =
        a.primitive_type() < b.primitive_type() ? a.primitive_type() : b.primitive_type();
    return less_subdart(m, primitive_type, a, b);
}
} // namespace wmtk::simplex::utils
