#include "AttributeType.hpp"
namespace wmtk::attribute {
const std::string_view attribute_type_name(AttributeType pt) {

    switch(pt) {
        case AttributeType::Char:
            return attribute_type_traits<AttributeType::Char>::name;
        case AttributeType::Int64:
            return attribute_type_traits<AttributeType::Int64>::name;
        case AttributeType::Double:
            return attribute_type_traits<AttributeType::Double>::name;
        case AttributeType::Rational:
            return attribute_type_traits<AttributeType::Rational>::name;
        default:
            break;
    }
    return "";
}

const std::string_view attribute_type_traits<AttributeType::Rational>::name = "Rational";
const std::string_view attribute_type_traits<AttributeType::Double>::name = "Double";
const std::string_view attribute_type_traits<AttributeType::Int64>::name = "Int64";
const std::string_view attribute_type_traits<AttributeType::Char>::name = "Char";
}
