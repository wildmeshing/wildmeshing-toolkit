#pragma once
#include "predicates.h"
#include <wmtk/Types.hpp>
#include <wmtk/utils/triangle_areas.hpp>

namespace wmtk::invariants::internal {
/// returns true if the simplex is oriented positively
template <typename T>
auto edge_orientation(const Vector<T, 2>& a, const Vector<T, 2>& b) -> bool
{
    return a <= b;
}

/// returns true if the simplex is oriented positively
template <typename T, bool UsePredicateIfPossible = true>
auto triangle_orientation(const Vector<T, 2>& a, const Vector<T, 2>& b, const Vector<T, 2>& c) -> bool
{
    if constexpr (UsePredicateIfPossible && std::is_same_v<double, T>) {
        return orient2d(const_cast<T*>(a.data()), const_cast<T*>(b.data()), const_cast<T*>(c.data())) > 0;
    } else {
        return utils::triangle_signed_2d_area(a, b, c) > 0;
    }
}

/// returns true if the simplex is oriented positively
template <typename T, bool UsePredicateIfPossible = true>
auto tetrahedron_orientation(
    const Vector<T, 3>& a,
    const Vector<T, 3>& b,
    const Vector<T, 3>& c,
    const Vector<T, 3>& d) -> bool
{
    if constexpr (UsePredicateIfPossible && std::is_same_v<double, T>) {
        return orient3d(const_cast<T*>(a.data()), const_cast<T*>(b.data()), const_cast<T*>(c.data()), const_cast<T*>(d.data())) > 0;
    } else {
        return utils::tetrahedron_signed_3d_area(a, b, c, d) > 0;
    }
}
} // namespace
