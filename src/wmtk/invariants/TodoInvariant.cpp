#include "TodoInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk {
TodoInvariant::TodoInvariant(
    const Mesh& m,
    const TypedAttributeHandle<int64_t>& todo_handle,
    const int64_t val)
    : Invariant(m, true, false, false)
    , m_todo_handle(todo_handle)
    , m_val(val)
{}

bool TodoInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == m_todo_handle.primitive_type());
    const attribute::Accessor<int64_t> split_todo_accessor =
        mesh().create_const_accessor<int64_t>(m_todo_handle);
    return split_todo_accessor.const_scalar_attribute(t) == m_val;
}

LargerInvariant::LargerInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& todo_handle,
    const double val)
    : Invariant(m, true, false, false)
    , m_todo_handle(todo_handle)
    , m_val(val)
{}

LargerInvariant::LargerInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& todo_handle,
    const TypedAttributeHandle<double>& comparison_handle,
    const double val)
    : Invariant(m, true, false, false)
    , m_todo_handle(todo_handle)
    , m_comparison_handle(comparison_handle)
    , m_val(val)
{}

bool LargerInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == m_todo_handle.primitive_type());
    const attribute::Accessor<double> split_todo_accessor =
        mesh().create_const_accessor<double>(m_todo_handle);

    if (m_comparison_handle.has_value()) {
        const attribute::Accessor<double> comp_accessor =
            mesh().create_const_accessor<double>(m_comparison_handle.value());
        return split_todo_accessor.const_scalar_attribute(t) >
               m_val * comp_accessor.const_scalar_attribute(t);
    } else {
        return split_todo_accessor.const_scalar_attribute(t) > m_val;
    }
}

SmallerInvariant::SmallerInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& todo_handle,
    const double val)
    : Invariant(m, true, false, false)
    , m_todo_handle(todo_handle)
    , m_val(val)
{}

SmallerInvariant::SmallerInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& todo_handle,
    const TypedAttributeHandle<double>& comparison_handle,
    const double val)
    : Invariant(m, true, false, false)
    , m_todo_handle(todo_handle)
    , m_comparison_handle(comparison_handle)
    , m_val(val)
{}

bool SmallerInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == m_todo_handle.primitive_type());
    const attribute::Accessor<double> split_todo_accessor =
        mesh().create_const_accessor<double>(m_todo_handle);

    if (m_comparison_handle.has_value()) {
        const attribute::Accessor<double> comp_accessor =
            mesh().create_const_accessor<double>(m_comparison_handle.value());
        return split_todo_accessor.const_scalar_attribute(t) <
               m_val * comp_accessor.const_scalar_attribute(t);
    } else {
        return split_todo_accessor.const_scalar_attribute(t) < m_val;
    }
}
} // namespace wmtk
