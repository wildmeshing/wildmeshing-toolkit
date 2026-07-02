#pragma once

#include <cctype>
#include <iostream>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>

#include "../ConnectedComponent.hpp"

namespace wmtk::components::simwild::expression_parser {

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

    /**
     * @brief Checks if the expression contains a logical NOT operation.
     */
    virtual bool contains_not() const { return false; }

    /**
     * @brief Checks if the expression contains a logical AND operation.
     */
    virtual bool contains_and() const { return false; }

    /**
     * @brief Checks if the expression contains a logical OR operation.
     */
    virtual bool contains_or() const { return false; }

    /**
     * @brief Checks if the expression contains only logical AND operations or none.
     */
    bool contains_only_and() const { return !contains_or() && !contains_not(); }
    /**
     * @brief Checks if the expression contains only logical OR operations or none.
     */
    bool contains_only_or() const { return !contains_and() && !contains_not(); }
    /**
     * @brief Checks if the expression contains only logical NOT operations or none.
     */
    bool contains_only_not() const { return !contains_and() && !contains_or(); }

    /**
     * @brief Returns the set of tags involved in this expression.
     *
     * This is useful for intent operations to determine which tags are relevant for the operation.
     *
     * @return CellTag The set of tags involved in this expression.
     */
    virtual CellTag tags_involved() const = 0;
    virtual std::set<std::string> tag_names_involved() const = 0;
};

using ExpressionPtr = std::unique_ptr<Expression>;

/**
 * An expression that evaluates to true if a specific tag is present.
 */
class TagExpr : public Expression
{
public:
    explicit TagExpr(int64_t tag, const std::string& name)
        : m_tag(tag)
        , m_name(name)
    {}

    bool eval(const CellTag& tags) const override { return tags.count(m_tag) != 0; }
    std::string to_string() const override { return "\"" + m_name + "\""; }
    CellTag tags_involved() const override { return {m_tag}; }
    std::set<std::string> tag_names_involved() const override { return {m_name}; }

private:
    int64_t m_tag;
    std::string m_name;
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
    CellTag tags_involved() const override { return {}; }
    std::set<std::string> tag_names_involved() const override { return {}; }
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

    bool contains_not() const override { return true; }
    bool contains_and() const override { return m_expr->contains_and(); }
    bool contains_or() const override { return m_expr->contains_or(); }
    CellTag tags_involved() const override { return m_expr->tags_involved(); }
    std::set<std::string> tag_names_involved() const override
    {
        return m_expr->tag_names_involved();
    }

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

    bool contains_not() const override { return m_left->contains_not() || m_right->contains_not(); }
    bool contains_and() const override { return true; }
    bool contains_or() const override { return m_left->contains_or() || m_right->contains_or(); }
    CellTag tags_involved() const override
    {
        CellTag tags = m_left->tags_involved();
        CellTag right_tags = m_right->tags_involved();
        tags.insert(right_tags.begin(), right_tags.end());
        return tags;
    }

    std::set<std::string> tag_names_involved() const override
    {
        auto names = m_left->tag_names_involved();
        auto right_names = m_right->tag_names_involved();
        names.insert(right_names.begin(), right_names.end());
        return names;
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

    bool contains_not() const override { return m_left->contains_not() || m_right->contains_not(); }
    bool contains_and() const override { return m_left->contains_and() || m_right->contains_and(); }
    bool contains_or() const override { return true; }
    CellTag tags_involved() const override
    {
        CellTag tags = m_left->tags_involved();
        CellTag right_tags = m_right->tags_involved();
        tags.insert(right_tags.begin(), right_tags.end());
        return tags;
    }
    std::set<std::string> tag_names_involved() const override
    {
        auto names = m_left->tag_names_involved();
        auto right_names = m_right->tag_names_involved();
        names.insert(right_names.begin(), right_names.end());
        return names;
    }

private:
    ExpressionPtr m_left;
    ExpressionPtr m_right;
};

} // namespace wmtk::components::simwild::expression_parser