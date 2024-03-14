
#pragma once
#include <wmtk/attribute/internal/CompoundAccessor.hpp>
#include <wmtk/utils/Rational.hpp>
#include "HybridRationalAttribute.hpp"

namespace wmtk::attribute::utils {


template <int D, typename MeshType>
class HybridRationalAccessor
    : public attribute::internal::CompoundAccessor<3, MeshType, char, wmtk::Rational, double>
{
public:
    using Base = attribute::internal::CompoundAccessor<3, MeshType, char, wmtk::Rational, double>;
    using Base::Base;
    using Base::const_value;
    using Base::get;
    using Base::value;

    HybridRationalAccessor(MeshType& m, const HybridRationalAttribute<D>& attr)
        : Base(m, std::get<0>(attr), std::get<1>(attr), std::get<2>(attr))
    {}
    void round(const Tuple& t)
    {
        auto [c, r, d] = value(t);
        d = r.unaryExpr([](const wmtk::Rational& v) { return v.to_double(); });
        c.setConstant(1);
    }

    void lift(const Tuple& t)
    {
        auto [c, r, d] = value(t);

        r = d.unaryExpr([](const double& v) { return wmtk::Rational(v); });
        c.setConstant(1);
    }

    auto& get_char_accessor() { return Base::template get<0>(); }
    const auto& get_char_const_accessor() const { return Base::template get<0>(); }

    auto& get_rational_accessor() { return Base::template get<1>(); }
    const auto& get_rational_const_accessor() const { return Base::template get<1>(); }

    auto& get_double_accessor() { return Base::template get<2>(); }
    const auto& get_double_const_accessor() const { return Base::template get<2>(); }

    auto char_value(const Tuple& t) { return get_char_accessor().vector_attribute(t); }
    auto char_const_value(const Tuple& t) const
    {
        return get_char_const_accessor().const_vector_attribute(t);
    }

    auto rational_value(const Tuple& t) { return get_rational_accessor().vector_attribute(t); }
    auto rational_const_value(const Tuple& t) const
    {
        return get_rational_const_accessor().const_vector_attribute(t);
    }

    auto double_value(const Tuple& t) { return get_double_accessor().vector_attribute(t); }
    auto double_const_value(const Tuple& t) const
    {
        return get_double_const_accessor().const_vector_attribute(t);
    }

    bool is_rounded(const Tuple& t) { return char_const_value(t) == 1; }


    template <size_t T>
    auto single_value(const Tuple& t)
    {}
};


template <int D, typename MeshType>
HybridRationalAccessor(MeshType& m, const HybridRationalAttribute<D>&)
    -> HybridRationalAccessor<D, MeshType>;

} // namespace wmtk::attribute::utils
