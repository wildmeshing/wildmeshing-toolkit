#pragma once

#include <optional>
#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk {
class TodoInvariant : public Invariant
{
    /**
     * Invariant for todo-list in scheduler. Recording which simplicity still need to be operated.
     * If the todo_tag tagged as 1 then return true, otherwise return false
     */
public:
    TodoInvariant(
        const Mesh& m,
        const TypedAttributeHandle<int64_t>& todo_handle,
        const int64_t val = 1);
    bool before(const simplex::Simplex& t) const override;

private:
    const TypedAttributeHandle<int64_t> m_todo_handle;
    const int64_t m_val;
};

class TodoLargerInvariant : public Invariant
{
    /**
     * Invariant for todo-list in scheduler. Recording which simplices still need to be operated.
     * If the todo_tag tagged as 1 then return true, otherwise return false
     */
public:
    TodoLargerInvariant(
        const Mesh& m,
        const TypedAttributeHandle<double>& todo_handle,
        const double val);

    TodoLargerInvariant(
        const Mesh& m,
        const TypedAttributeHandle<double>& todo_handle,
        const TypedAttributeHandle<double>& comparison_handle,
        const double pre_factor = 1);

    bool before(const simplex::Simplex& t) const override;

private:
    const TypedAttributeHandle<double> m_todo_handle;
    const std::optional<TypedAttributeHandle<double>> m_comparison_handle;
    const double m_val;
};

class TodoSmallerInvariant : public Invariant
{
    /**
     * Invariant for todo-list in scheduler. Recording which simplices still need to be operated.
     * If the todo_tag tagged as 1 then return true, otherwise return false
     */
public:
    TodoSmallerInvariant(
        const Mesh& m,
        const TypedAttributeHandle<double>& todo_handle,
        const double val);

    TodoSmallerInvariant(
        const Mesh& m,
        const TypedAttributeHandle<double>& todo_handle,
        const TypedAttributeHandle<double>& comparison_handle,
        const double pre_factor = 1);

    bool before(const simplex::Simplex& t) const override;

private:
    const TypedAttributeHandle<double> m_todo_handle;
    const std::optional<TypedAttributeHandle<double>> m_comparison_handle;
    const double m_val;
};
} // namespace wmtk
