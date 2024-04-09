#pragma once

#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk {
template <typename T>
class SimplexInversionInvariant : public Invariant
{
    SimplexInversionInvariant(const Mesh& m, const TypedAttributeHandle<T>& coordinate)
        : Invariant(m, true, false, true)
    {}

    bool after(const std::vector<Tuple>&, const std::vector<Tuple>& top_dimension_tuples_after)
        const override
    {
        throw std::runtime_error("Invalid type");
    }
};

template <>
class SimplexInversionInvariant<double> : public Invariant
{
public:
    SimplexInversionInvariant(const Mesh& m, const TypedAttributeHandle<double>& coordinate);
    using Invariant::Invariant;

    /**
     *  we assume with local vid order (v0,v1,v2,v3) has positive volume (orient3d(v0,v1,v2,v3)>0)
     a CLOCKWISE input tuple will have the same orientation as the local vid order
     */
    bool after(const std::vector<Tuple>&, const std::vector<Tuple>& top_dimension_tuples_after)
        const override;

private:
    const TypedAttributeHandle<double> m_coordinate_handle;
};

template <>
class SimplexInversionInvariant<Rational> : public Invariant
{
public:
    SimplexInversionInvariant(const Mesh& m, const TypedAttributeHandle<Rational>& coordinate);
    using Invariant::Invariant;

    /**
     *  we assume with local vid order (v0,v1,v2,v3) has positive volume (orient3d(v0,v1,v2,v3)>0)
     a CLOCKWISE input tuple will have the same orientation as the local vid order
     */
    bool after(const std::vector<Tuple>&, const std::vector<Tuple>& top_dimension_tuples_after)
        const override;

private:
    const TypedAttributeHandle<Rational> m_coordinate_handle;
};
} // namespace wmtk
