#pragma once

#include "Expression.hpp"

namespace wmtk::components::image_simulation::expression_parser {

class Parser
{
public:
    /**
     * @brief Constructs a new Parser object.
     *
     * @param input The input string containing the expression to parse.
     * @param tag_name_to_id A map from tag names to their respective IDs.
     */
    explicit Parser(const std::string& input, const std::map<std::string, int64_t>& tag_name_to_id)
        : m_input(input)
        , m_tag_name_to_id(tag_name_to_id)
    {}

    /**
     * @brief Parses the input string into a boolean expression tree.
     *
     * @return ExpressionPtr The root of the parsed expression tree.
     */
    ExpressionPtr parse();

    /**
     * @brief Parses a logical OR expression.
     *
     * @return ExpressionPtr The parsed OR expression, or lower precedence expression.
     */
    ExpressionPtr parseOr();

    /**
     * @brief Parses a logical AND expression.
     *
     * @return ExpressionPtr The parsed AND expression, or lower precedence expression.
     */
    ExpressionPtr parseAnd();

    /**
     * @brief Parses a unary expression (e.g., logical NOT).
     *
     * @return ExpressionPtr The parsed unary expression, or lower precedence expression.
     */
    ExpressionPtr parseUnary();

    /**
     * @brief Parses a primary expression (variables, parenthesized expressions).
     *
     * @return ExpressionPtr The parsed primary expression.
     */
    ExpressionPtr parsePrimary();

    /**
     * @brief Parses an identifier from the input string.
     *
     * @return std::string The parsed identifier.
     */
    std::string parseIdentifier();

    /**
     * @brief Checks if the current character matches a given character, advancing the internal
     * pointer if it does.
     *
     * @param c The character to match.
     * @return true If the current character matches `c`.
     * @return false Otherwise.
     */
    bool match(char c);

    /**
     * @brief Skips whitespaces in the input string.
     */
    void skipWhitespace();

private:
    std::string m_input; // The input string to parse.
    size_t m_pos = 0; // Current position in the input string.

    std::map<std::string, int64_t> m_tag_name_to_id;
};

/**
 * @brief Parses a boolean expression from a string input.
 *
 * @param input The input string containing the expression to parse.
 * @param tag_name_to_id A map from tag names to their respective IDs.
 * @return ExpressionPtr The root of the parsed expression tree.
 */
ExpressionPtr parse(const std::string& input, const std::map<std::string, int64_t>& tag_name_to_id);

} // namespace wmtk::components::image_simulation::expression_parser