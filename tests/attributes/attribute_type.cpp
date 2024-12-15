#include <numeric>

#include <catch2/catch_test_macros.hpp>
#include <wmtk/attribute/AttributeType.hpp>


using namespace wmtk::attribute;

TEST_CASE("test_attribute_type_names", "[attributes]")
{
    using AT = AttributeType;
    CHECK(attribute_type_name(AT::Char) == "Char");
    CHECK(attribute_type_name(AT::Double) == "Double");
    CHECK(attribute_type_name(AT::Int64) == "Int64");
    CHECK(attribute_type_name(AT::Rational) == "Rational");

}

