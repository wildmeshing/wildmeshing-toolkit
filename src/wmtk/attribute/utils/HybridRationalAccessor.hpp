#pragma once
#include <wmtk/attribute/internal/CompoundAccessor.hpp>
#include <wmtk/utils/Rational.hpp>
#include "HybridRationalAttribute.hpp"

namespace wmtk::simplex {
class Simplex;
}
namespace wmtk::attribute::utils {


template <int D = Eigen::Dynamic, typename MeshType = wmtk::Mesh>
class HybridRationalAccessor
    : public attribute::internal::CompoundAccessor<3, MeshType, char, wmtk::Rational, double>
{
public:
    using Base = attribute::internal::CompoundAccessor<3, MeshType, char, wmtk::Rational, double>;
    using Base::Base;
    using Base::const_value;
    using Base::get;
    using Base::value;
    // struct HybridValue : public Base::ValueType
    //{
    // };

    HybridRationalAccessor(MeshType& m, const HybridRationalAttribute<D>& attr)
        : Base(m, std::get<0>(attr), std::get<1>(attr), std::get<2>(attr))
    {}

    HybridRationalAccessor(const MeshAttributeHandle& handle)
        : HybridRationalAccessor(
              static_cast<MeshType&>(const_cast<wmtk::Mesh&>(handle.mesh())),
              handle.template as_from_held_type<MeshAttributeHandle::HeldType::HybridRational>())
    {
        assert(handle.holds_from_held_type<MeshAttributeHandle::HeldType::HybridRational>());
    }

    PrimitiveType primitive_type() const { return Base::template primitive_type<0>(); }


    void round(const Tuple& t, bool enable_double = false)
    {
        auto [c, r, d] = value(t);
        d = r.unaryExpr([](const wmtk::Rational& v) { return v.to_double(); });
        if (enable_double) {
            c.setConstant(1);
            // TODO: invalidate the rational value
        }
    }

    void lift(const Tuple& t, bool enable_double = false)
    {
        auto [c, r, d] = value(t);

        r = d.unaryExpr([](const double& v) { return wmtk::Rational(v); });
        if (enable_double) {
            c.setConstant(1);
        }
    }

    auto& get_char_accessor() { return Base::template get<0>(); }
    const auto& get_char_const_accessor() const { return Base::template get<0>(); }

    auto& get_rational_accessor() { return Base::template get<1>(); }
    const auto& get_rational_const_accessor() const { return Base::template get<1>(); }

    auto& get_double_accessor() { return Base::template get<2>(); }
    const auto& get_double_const_accessor() const { return Base::template get<2>(); }

    auto raw_char_value(const Tuple& t) { return get_char_accessor().vector_attribute(t); }
    auto raw_char_const_value(const Tuple& t) const
    {
        return get_char_const_accessor().const_vector_attribute(t);
    }

    auto raw_rational_value(const Tuple& t) { return get_rational_accessor().vector_attribute(t); }
    auto raw_rational_const_value(const Tuple& t) const
    {
        return get_rational_const_accessor().const_vector_attribute(t);
    }

    auto raw_double_value(const Tuple& t) { return get_double_accessor().vector_attribute(t); }
    auto raw_double_const_value(const Tuple& t) const
    {
        return get_double_const_accessor().const_vector_attribute(t);
    }

    bool is_rounded(const Tuple& t) const { return (raw_char_const_value(t).array() == 1).all(); }

    // checks if every simplex or tuple is rounded
    template <typename Iterable>
    bool are_all_rounded(const Iterable& tuples) const
    {
        constexpr static bool is_tuple = std::is_same_v<typename Iterable::value_type, Tuple>;
        constexpr static bool is_simplex =
            !is_tuple && std::is_same_v<typename Iterable::value_type, wmtk::simplex::Simplex>;
        static_assert(is_tuple || is_simplex);
        bool all_rounded = true;
        for (const auto& tup : tuples) {
            if constexpr (is_tuple) {
                if (!is_rounded(tup)) {
                    all_rounded = false;
                    break;
                }
            } else if constexpr (is_simplex) {
                if (!is_rounded(tup.tuple())) {
                    all_rounded = false;
                    break;
                }
            }
        }
        return all_rounded;
    }

    // template <size_t T>
    // auto single_value(const Tuple& t)
    //{
    //     return
    // }
};


template <int D, typename MeshType>
HybridRationalAccessor(MeshType& m, const HybridRationalAttribute<D>&)
    -> HybridRationalAccessor<D, MeshType>;

// if we're passed in a MeshAttributeHandle we should default to the type assumed by the
// meshattributehandle held type
HybridRationalAccessor(const MeshAttributeHandle&) -> HybridRationalAccessor<
    MeshAttributeHandle::held_handle_type<MeshAttributeHandle::HeldType::HybridRational>::Dim,
    wmtk::Mesh>;

} // namespace wmtk::attribute::utils

