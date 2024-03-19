#pragma once
#include <spdlog/fmt/fmt.h> // consistent way to obtain fmt::format if we are depending on spdlog
#include <Eigen/Core>
#include <tuple>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::attribute::utils {


template <int D = Eigen::Dynamic>
class HybridRationalAttribute : public std::tuple<
                                    TypedAttributeHandle<char>,
                                    TypedAttributeHandle<wmtk::Rational>,
                                    TypedAttributeHandle<double>>
{
public:
    using TupleType = tuple<
        TypedAttributeHandle<char>,
        TypedAttributeHandle<wmtk::Rational>,
        TypedAttributeHandle<double>>;
    using TupleType::TupleType;
    using Type = TupleType;
    using value_type = Type;

    template <size_t Index>
    auto get() const
    {
        return std::get<Index>(static_cast<const TupleType&>(*this));
    }
    template <size_t Index>
    auto get()
    {
        return std::get<Index>(static_cast<TupleType&>(*this));
    }

    auto get_char() const { return get<0>(); }

    auto get_rational() const { return get<1>(); }

    auto get_double() const { return get<2>(); }

    PrimitiveType primitive_type() const { return get_char().primitive_type(); }

public:
    template <typename MeshType>
    static HybridRationalAttribute<D> register_attribute(
        MeshType& mesh,
        const std::string& name,
        PrimitiveType pt,
        int64_t size = 1,
        const wmtk::Rational& default_value = 0.0);
};

template <int D>
template <typename MeshType>
HybridRationalAttribute<D> HybridRationalAttribute<D>::register_attribute(
    MeshType& mesh,
    const std::string& name,
    PrimitiveType pt,
    int64_t size,
    const wmtk::Rational& default_value)
{
    const std::string char_name = fmt::format("[hybrid_rational](char){}", name);
    const std::string double_name = fmt::format("[hybrid_rational](double){}", name);
    const std::string rational_name = fmt::format("[hybrid_rational](rational){}", name);
    auto char_attr = mesh.template register_attribute<char>(
        char_name,
        pt,
        size,
        false,
        /*default_value=*/false);
    auto double_attr = mesh.template register_attribute<double>(
        double_name,
        pt,
        size,
        false,
        /*default_value=*/default_value.to_double());
    auto rational_attr = mesh.template register_attribute<wmtk::Rational>(
        rational_name,
        pt,
        size,
        false,
        /*default_value=*/default_value);
    return HybridRationalAttribute<D>(char_attr, rational_attr, double_attr);
}

} // namespace wmtk::attribute::utils
