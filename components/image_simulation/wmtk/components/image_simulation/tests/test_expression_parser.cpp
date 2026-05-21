#include <catch2/catch_test_macros.hpp>
#include <wmtk/components/image_simulation/expression_parser/Parser.hpp>

using namespace wmtk;
using namespace components::image_simulation;
using namespace expression_parser;

std::map<std::string, int64_t> tag_name_to_id = {{"A", 1}, {"B", 2}, {"C", 3}};

CellTag tags1 = {1}; // A
CellTag tags2 = {2}; // B
CellTag tags3 = {3}; // C
CellTag tags4 = {1, 2}; // A and B
CellTag tags5 = {1, 3}; // A and C
CellTag tags6 = {2, 3}; // B and C
CellTag tags7 = {1, 2, 3}; // A, B, and C
CellTag tags8 = {}; // empty

TEST_CASE("test_parser1", "[image_simulation],[expression_parser]")
{
    Parser parser("A & (B | !C)", tag_name_to_id);
    const auto expr = parser.parse();
    // logger().info("Expression string: {}", expr->to_string());

    CHECK(expr->eval(tags1));
    CHECK_FALSE(expr->eval(tags2));
    CHECK_FALSE(expr->eval(tags3));
    CHECK(expr->eval(tags4));
    CHECK_FALSE(expr->eval(tags5));
    CHECK_FALSE(expr->eval(tags6));
    CHECK(expr->eval(tags7));
}

TEST_CASE("test_parser2", "[image_simulation],[expression_parser]")
{
    Parser parser("A & B & C", tag_name_to_id);
    const auto expr = parser.parse();
    // logger().info("Expression string: {}", expr->to_string());

    CHECK_FALSE(expr->eval(tags1));
    CHECK_FALSE(expr->eval(tags2));
    CHECK_FALSE(expr->eval(tags3));
    CHECK_FALSE(expr->eval(tags4));
    CHECK_FALSE(expr->eval(tags5));
    CHECK_FALSE(expr->eval(tags6));
    CHECK(expr->eval(tags7));
}

TEST_CASE("test_parser3", "[image_simulation],[expression_parser]")
{
    Parser parser("_", tag_name_to_id);
    const auto expr = parser.parse();
    // logger().info("Expression string: {}", expr->to_string());

    CHECK_FALSE(expr->eval(tags1));
    CHECK_FALSE(expr->eval(tags2));
    CHECK_FALSE(expr->eval(tags3));
    CHECK_FALSE(expr->eval(tags4));
    CHECK_FALSE(expr->eval(tags5));
    CHECK_FALSE(expr->eval(tags6));
    CHECK_FALSE(expr->eval(tags7));
    CHECK(expr->eval(tags8));
}

TEST_CASE("test_parser4", "[image_simulation],[expression_parser]")
{
    Parser parser("!(_ | C)", tag_name_to_id);
    const auto expr = parser.parse();
    // logger().info("Expression string: {}", expr->to_string());

    CHECK(expr->eval(tags1));
    CHECK(expr->eval(tags2));
    CHECK_FALSE(expr->eval(tags3));
    CHECK(expr->eval(tags4));
    CHECK_FALSE(expr->eval(tags5));
    CHECK_FALSE(expr->eval(tags6));
    CHECK_FALSE(expr->eval(tags7));
    CHECK_FALSE(expr->eval(tags8));
}

TEST_CASE("test_parser5", "[image_simulation],[expression_parser]")
{
    Parser parser("A | B & C", tag_name_to_id);
    const auto expr = parser.parse();
    // logger().info("Expression string: {}", expr->to_string());

    CHECK(expr->eval(tags1));
    CHECK_FALSE(expr->eval(tags2));
    CHECK_FALSE(expr->eval(tags3));
    CHECK(expr->eval(tags4));
    CHECK(expr->eval(tags5));
    CHECK(expr->eval(tags6));
    CHECK(expr->eval(tags7));
    CHECK_FALSE(expr->eval(tags8));
}