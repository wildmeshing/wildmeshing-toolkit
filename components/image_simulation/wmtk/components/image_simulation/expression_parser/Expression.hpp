#pragma once

#include <cctype>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>

#include "../ConnectedComponent.hpp"

namespace wmtk::components::image_simulation::expression_parser {

/**
 * @brief Base class for boolean expression tree nodes used to evaluate cell tags.
 */
class Expression
{
public:
    virtual ~Expression() = default;

    /**
     * @brief Evaluates the expression against a given set of tags.
     *
     * @param tags The tags evaluated against the boolean expression.
     * @return true If the expression is satisfied by the provided tags.
     * @return false Otherwise.
     */
    virtual bool eval(const CellTag& tags) const = 0;

    /**
     * @brief Converts the expression to its string representation.
     * Mainly for debugging purposes.
     *
     * @return std::string The string representation of the expression.
     */
    virtual std::string to_string() const = 0;
};

using ExpressionPtr = std::unique_ptr<Expression>;

/**
 * An expression that evaluates to true if a specific tag is present.
 */
class TagExpr : public Expression
{
public:
    explicit TagExpr(int64_t tag)
        : m_tag(tag)
    {}

    bool eval(const CellTag& tags) const override { return tags.count(m_tag) != 0; }
    std::string to_string() const override { return "\"" + std::to_string(m_tag) + "\""; }

private:
    int64_t m_tag;
};

/**
 * An expression that evaluates to true if the provided tags set is empty.
 */
class EmptyExpr : public Expression
{
public:
    EmptyExpr() = default;

    bool eval(const CellTag& tags) const override { return tags.empty(); }
    std::string to_string() const override { return "\"_\""; }
};

/**
 * A logical NOT expression.
 */
class NotExpr : public Expression
{
public:
    explicit NotExpr(ExpressionPtr expr)
        : m_expr(std::move(expr))
    {}

    bool eval(const CellTag& tags) const override { return !m_expr->eval(tags); }
    std::string to_string() const override { return "!(" + m_expr->to_string() + ")"; }

private:
    ExpressionPtr m_expr;
};

/**
 * A logical AND expression.
 */
class AndExpr : public Expression
{
public:
    AndExpr(ExpressionPtr left, ExpressionPtr right)
        : m_left(std::move(left))
        , m_right(std::move(right))
    {}

    bool eval(const CellTag& tags) const override
    {
        return m_left->eval(tags) && m_right->eval(tags);
    }
    std::string to_string() const override
    {
        return "(" + m_left->to_string() + " & " + m_right->to_string() + ")";
    }

private:
    ExpressionPtr m_left;
    ExpressionPtr m_right;
};

/**
 * A logical OR expression.
 */
class OrExpr : public Expression
{
public:
    OrExpr(ExpressionPtr left, ExpressionPtr right)
        : m_left(std::move(left))
        , m_right(std::move(right))
    {}

    bool eval(const CellTag& tags) const override
    {
        return m_left->eval(tags) || m_right->eval(tags);
    }

    std::string to_string() const override
    {
        return "(" + m_left->to_string() + " | " + m_right->to_string() + ")";
    }

private:
    ExpressionPtr m_left;
    ExpressionPtr m_right;
};

} // namespace wmtk::components::image_simulation::expression_parser