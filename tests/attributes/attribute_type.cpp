#include <numeric>

#include <catch2/catch_test_macros.hpp>
#include <wmtk/attribute/AttributeType.hpp>


using namespace wmtk::attribute;

TEST_CASE("test_attribute_type_names", "[attributes]")
{
    using AT = AttributeType;
    // converting to string because some compilers fail with this combo of catch + string_view comparisons?
    CHECK(std::string(attribute_type_name(AT::Char)) == "Char");
    CHECK(std::string(attribute_type_name(AT::Double)) == "Double");
    CHECK(std::string(attribute_type_name(AT::Int64)) == "Int64");
    CHECK(std::string(attribute_type_name(AT::Rational)) == "Rational");

}

