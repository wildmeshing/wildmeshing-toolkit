#include "TodoInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk {
TodoInvariant::TodoInvariant(
    const Mesh& m,
    const TypedAttributeHandle<int64_t>& todo_handle,
    const int64_t val)
    : Invariant(m)
    , m_todo_handle(todo_handle)
    , m_val(val)
{}

bool TodoInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == m_todo_handle.primitive_type());
    ConstAccessor<int64_t> split_todo_accessor = mesh().create_accessor<int64_t>(m_todo_handle);
    return split_todo_accessor.const_scalar_attribute(t.tuple()) == m_val;
}

TodoLargerInvariant::TodoLargerInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& todo_handle,
    const double val)
    : Invariant(m)
    , m_todo_handle(todo_handle)
    , m_val(val)
{}

bool TodoLargerInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == m_todo_handle.primitive_type());
    ConstAccessor<double> split_todo_accessor = mesh().create_accessor<double>(m_todo_handle);
    return split_todo_accessor.const_scalar_attribute(t.tuple()) > m_val;
}

TodoSmallerInvariant::TodoSmallerInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& todo_handle,
    const double val)
    : Invariant(m)
    , m_todo_handle(todo_handle)
    , m_val(val)
{}

bool TodoSmallerInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == m_todo_handle.primitive_type());
    ConstAccessor<double> split_todo_accessor = mesh().create_accessor<double>(m_todo_handle);
    return split_todo_accessor.const_scalar_attribute(t.tuple()) < m_val;
}
} // namespace wmtk
