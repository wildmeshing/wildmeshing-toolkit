#pragma once

#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk {
class RoundedInvariant : public Invariant
{
public:
    /**
     * @brief Construct a new Rounded Invariant object
     *
     * @param m
     * @param coordinate
     * @param inverse if set to false, return true if rounded; if set to true, return false if
     * rounded
     */
    RoundedInvariant(
        const Mesh& m,
        const attribute::TypedAttributeHandle<Rational>& coordinate,
        bool inverse_flag = false);
    using Invariant::Invariant;

    bool before(const simplex::Simplex& t) const override;

private:
    const attribute::TypedAttributeHandle<Rational> m_coordinate_handle;
    bool inverse = false;
};
} // namespace wmtk