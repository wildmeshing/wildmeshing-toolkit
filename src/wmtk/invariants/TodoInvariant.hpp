#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
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
        const MeshAttributeHandle<int64_t>& todo_handle,
        const int64_t val = 1);
    bool before(const simplex::Simplex& t) const override;

private:
    const MeshAttributeHandle<int64_t> m_todo_handle;
    const int64_t m_val;
};

class TodoLargerInvariant : public Invariant
{
    /**
     * Invariant for todo-list in scheduler. Recording which simplicity still need to be operated.
     * If the todo_tag tagged as 1 then return true, otherwise return false
     */
public:
    TodoLargerInvariant(
        const Mesh& m,
        const MeshAttributeHandle<double>& todo_handle,
        const double val);
    bool before(const simplex::Simplex& t) const override;

private:
    const MeshAttributeHandle<double> m_todo_handle;
    const double m_val;
};

class TodoSmallerInvariant : public Invariant
{
    /**
     * Invariant for todo-list in scheduler. Recording which simplicity still need to be operated.
     * If the todo_tag tagged as 1 then return true, otherwise return false
     */
public:
    TodoSmallerInvariant(
        const Mesh& m,
        const MeshAttributeHandle<double>& todo_handle,
        const double val);
    bool before(const simplex::Simplex& t) const override;

private:
    const MeshAttributeHandle<double> m_todo_handle;
    const double m_val;
};
} // namespace wmtk