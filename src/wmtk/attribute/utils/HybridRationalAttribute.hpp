#pragma once
#include <spdlog/fmt/fmt.h> // consistent way to obtain fmt::format if we are depending on spdlog
#include <Eigen/Core>
#include <tuple>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include <wmtk/attribute/internal/MapTypes.hpp>
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
    using Type = std::tuple<char, wmtk::Rational, double>;
    using value_type = Type;
    constexpr static int Dim = D;
    using MapValueType = std::tuple<
        wmtk::attribute::internal::MapResult<char, D>,
        wmtk::attribute::internal::MapResult<wmtk::Rational, D>,
        wmtk::attribute::internal::MapResult<double, D>>;
    using ConstMapValueType = std::tuple<
        wmtk::attribute::internal::ConstMapResult<char, D>,
        wmtk::attribute::internal::ConstMapResult<wmtk::Rational, D>,
        wmtk::attribute::internal::ConstMapResult<double, D>>;

    using ResultValueType = std::tuple<
        wmtk::attribute::internal::VectorResult<char, D>,
        wmtk::attribute::internal::VectorResult<wmtk::Rational, D>,
        wmtk::attribute::internal::VectorResult<double, D>>;

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
    // builds the attributes used to represent a hybrid rational attribute
    // the name provided ise prepended with type info to have the formm
    // "[hybrid_rational({type}){name}"
    template <typename MeshType>
    static HybridRationalAttribute<D> register_attribute(
        MeshType& mesh,
        const std::string& name,
        PrimitiveType pt,
        int64_t size = 1,
        const wmtk::Rational& default_value = 0.0);

    // builds auxilliary attributes to enable rounding from rational.
    // The double attribute passed in is the one returned
    // note that because the data is generated from double rounding must already have happened
    template <typename MeshType>
    static HybridRationalAttribute<D> register_attribute_from_double(
        MeshType& mesh,
        const TypedAttributeHandle<double>& handle,
        const std::string& name = {});
    // builds auxilliary attributes to enable rounding from rational.
    // The rational attribute passed in is the one returned
    // if load_rounded_values is true then the rational data will be copied to the double as
    // possible if set_rounded is true then the roundedness flag will also be set - this should only
    // be allowed if load_rounded_values is set to true
    template <typename MeshType>
    static HybridRationalAttribute<D> register_attribute_from_rational(
        MeshType& mesh,
        const TypedAttributeHandle<wmtk::Rational>& handle,
        const std::string& name = {},
        bool load_rounded_values = true,
        bool set_rounded = false);
};

template <int D, typename MeshType>
class HybridRationalAccessor;
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

template <int D>
template <typename MeshType>
HybridRationalAttribute<D> HybridRationalAttribute<D>::register_attribute_from_double(
    MeshType& mesh,
    const TypedAttributeHandle<double>& handle,
    const std::string& name)
{
    const std::string char_name = fmt::format("[hybrid_rational](char){}", name);
    const std::string rational_name = fmt::format("[hybrid_rational](rational){}", name);
    double default_value = mesh.get_attribute_default_value(handle);
    const PrimitiveType pt = handle.primitive_type();
    const int64_t size = mesh.get_attribute_dimension(handle);
    auto char_attr = mesh.template register_attribute<char>(
        char_name,
        pt,
        size,
        false,
        /*default_value=*/false);
    auto rational_attr = mesh.template register_attribute<wmtk::Rational>(
        rational_name,
        pt,
        size,
        false,
        /*default_value=*/default_value);

    HybridRationalAttribute<D> ret(char_attr, rational_attr, handle);
    // copy the data from double to rational
    HybridRationalAccessor<D, MeshType> hac(mesh, ret);
    for (const auto& t : mesh.get_all(handle.primitive_type())) {
        hac.lift(t, true);
    }
    // because this is already rounded also fill char with true
    return ret;
}

template <int D>
template <typename MeshType>
HybridRationalAttribute<D> HybridRationalAttribute<D>::register_attribute_from_rational(
    MeshType& mesh,
    const TypedAttributeHandle<wmtk::Rational>& handle,
    const std::string& name,
    bool load_rounded_values,
    bool set_rounded)
{
    const std::string char_name = fmt::format("[hybrid_rational](char){}", name);
    const std::string double_name = fmt::format("[hybrid_rational](double){}", name);
    const wmtk::Rational& default_value = mesh.get_attribute_default_value(handle);
    const PrimitiveType pt = handle.primitive_type();
    const int64_t size = mesh.get_attribute_dimension(handle);
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

    HybridRationalAttribute<D> ret(char_attr, handle, double_attr);
    if (load_rounded_values) {
        HybridRationalAccessor<D, MeshType> hac(mesh, ret);
        for (const auto& t : mesh.get_all(handle.primitive_type())) {
            hac.round(t, set_rounded);
        }
    } else {
        assert(!set_rounded); // can only specify attributes as rounded if data was fed
                              // into the double attribute
    }
    return ret;
}
} // namespace wmtk::attribute::utils

// template <size_t Index, int D, typename MeshType>
// size_t std::get(const wmtk::attribute::utils::HybridRationalAttribute<D>& attr)
//{
//     return 0;
//    // static_cast <
//}
