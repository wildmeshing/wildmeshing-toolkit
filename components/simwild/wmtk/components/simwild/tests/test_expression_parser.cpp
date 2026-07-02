#include <catch2/catch_test_macros.hpp>
#include <wmtk/components/simwild/expression_parser/Parser.hpp>

using namespace wmtk;
using namespace components::simwild;
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

TEST_CASE("test_parser_all", "[simwild],[expression_parser]")
{
    const auto expr = parse("A & (B | !C)", tag_name_to_id);
    // logger().info("Expression string: {}", expr->to_string());

    CHECK(expr->eval(tags1));
    CHECK_FALSE(expr->eval(tags2));
    CHECK_FALSE(expr->eval(tags3));
    CHECK(expr->eval(tags4));
    CHECK_FALSE(expr->eval(tags5));
    CHECK_FALSE(expr->eval(tags6));
    CHECK(expr->eval(tags7));
}

TEST_CASE("test_parser_and", "[simwild],[expression_parser]")
{
    const auto expr = parse("A & B & C", tag_name_to_id);
    // logger().info("Expression string: {}", expr->to_string());

    CHECK_FALSE(expr->eval(tags1));
    CHECK_FALSE(expr->eval(tags2));
    CHECK_FALSE(expr->eval(tags3));
    CHECK_FALSE(expr->eval(tags4));
    CHECK_FALSE(expr->eval(tags5));
    CHECK_FALSE(expr->eval(tags6));
    CHECK(expr->eval(tags7));
}

TEST_CASE("test_parser_empty", "[simwild],[expression_parser]")
{
    const auto expr = parse("_", tag_name_to_id);
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

TEST_CASE("test_parser_not_empty", "[simwild],[expression_parser]")
{
    const auto expr = parse("!(_ | C)", tag_name_to_id);
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

TEST_CASE("test_parser_priority", "[simwild],[expression_parser]")
{
    // should evaluate as A | (B & C), not (A | B) & C
    const auto expr = parse("A | B & C", tag_name_to_id);
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

TEST_CASE("test_parser_contains", "[simwild],[expression_parser]")
{
    const auto expr = parse("A & (B | !C)", tag_name_to_id);
    // logger().info("Expression string: {}", expr->to_string());

    CHECK(expr->contains_not());
    CHECK(expr->contains_and());
    CHECK(expr->contains_or());

    const auto expr2 = parse("A & B & C", tag_name_to_id);
    // logger().info("Expression string: {}", expr2->to_string());

    CHECK_FALSE(expr2->contains_not());
    CHECK(expr2->contains_and());
    CHECK_FALSE(expr2->contains_or());
    CHECK(expr2->contains_only_and());
    CHECK_FALSE(expr2->contains_only_or());
    CHECK_FALSE(expr2->contains_only_not());

    const auto expr3 = parse("_", tag_name_to_id);
    // logger().info("Expression string: {}", expr3->to_string());

    CHECK_FALSE(expr3->contains_not());
    CHECK_FALSE(expr3->contains_and());
    CHECK_FALSE(expr3->contains_or());
    CHECK(expr3->contains_only_and());
    CHECK(expr3->contains_only_or());
    CHECK(expr3->contains_only_not());
}

TEST_CASE("test_parser_tags_involved", "[simwild],[expression_parser]")
{
    const auto expr = parse("A & (B | !C)", tag_name_to_id);
    logger().info("Expression string: {}", expr->to_string());

    CellTag tags_involved = expr->tags_involved();
    CHECK(tags_involved.size() == 3);
    CHECK(tags_involved.count(1) == 1); // A
    CHECK(tags_involved.count(2) == 1); // B
    CHECK(tags_involved.count(3) == 1); // C

    auto tag_names_involved = expr->tag_names_involved();
    CHECK(tag_names_involved.size() == 3);
    CHECK(tag_names_involved.count("A") == 1);
    CHECK(tag_names_involved.count("B") == 1);
    CHECK(tag_names_involved.count("C") == 1);

    const auto expr2 = parse("_", tag_name_to_id);
    logger().info("Expression string: {}", expr2->to_string());

    CellTag tags_involved2 = expr2->tags_involved();
    CHECK(tags_involved2.empty());
    auto tag_names_involved2 = expr2->tag_names_involved();
    CHECK(tag_names_involved2.empty());
}