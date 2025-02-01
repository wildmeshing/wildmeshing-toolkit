#pragma once
#include <wmtk/utils/Rational.hpp>

namespace wmtk::attribute {
enum class AttributeType { Char = 0, Int64 = 1, Double = 2, Rational = 3 };

template <AttributeType AT>
struct type_from_attribute_type_enum
{
};
template <>
struct type_from_attribute_type_enum<AttributeType::Char>
{
    using type = char;
};
template <>
struct type_from_attribute_type_enum<AttributeType::Double>
{
    using type = double;
};
template <>
struct type_from_attribute_type_enum<AttributeType::Int64>
{
    using type = int64_t;
};
template <>
struct type_from_attribute_type_enum<AttributeType::Rational>
{
    using type = wmtk::Rational;
};

template <AttributeType AT>
using type_from_attribute_type_enum_t = typename type_from_attribute_type_enum<AT>::type;

template <typename T>
inline constexpr auto attribute_type_enum_from_type() -> AttributeType
{
    if constexpr (std::is_same_v<T, char>) {
        return AttributeType::Char;
    } else if constexpr (std::is_same_v<T, double>) {
        return AttributeType::Double;
    } else if constexpr (std::is_same_v<T, int64_t>) {
        return AttributeType::Int64;
    } else if constexpr (std::is_same_v<T, wmtk::Rational>) {
        return AttributeType::Rational;
    }
    // If a compiler complains about the potentiality of no return value then a type accepted by the
    // HAndleVariant is not being represented properly. If the comppiler is simply unhappy to not
    // see a return then we should hack a default return value in an else statement with an asswert
    // :(.
    else {
        static_assert(std::is_same_v<T, char>);
        return AttributeType::Char;
    }
}
} // namespace wmtk::attribute
