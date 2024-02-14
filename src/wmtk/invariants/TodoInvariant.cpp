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
    return split_todo_accessor.const_scalar_attribute(t.tuple()) == m_val;
}

TodoLargerInvariant::TodoLargerInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& todo_handle,
    const double val)
    : Invariant(m, true, false, false)
    , m_todo_handle(todo_handle)
    , m_val(val)
{}

bool TodoLargerInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == m_todo_handle.primitive_type());
    const attribute::Accessor<double> split_todo_accessor =
        mesh().create_const_accessor<double>(m_todo_handle);
    return split_todo_accessor.const_scalar_attribute(t.tuple()) > m_val;
}

TodoAvgEnergyLargerInvariant::TodoAvgEnergyLargerInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& todo_handle,
    const double val)
    : Invariant(m)
    , m_todo_handle(todo_handle)
    , m_val(val)
{}

bool TodoAvgEnergyLargerInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == PrimitiveType::Edge);
    assert(m_todo_handle.primitive_type() == PrimitiveType::Triangle);
    const attribute::Accessor<double> split_todo_accessor =
        mesh().create_const_accessor<double>(m_todo_handle);
    if (mesh().is_boundary(t)) {
        return split_todo_accessor.const_scalar_attribute(t.tuple()) > m_val;
    }

    auto res = split_todo_accessor.const_scalar_attribute(t.tuple());
    res += split_todo_accessor.const_scalar_attribute(
        mesh().switch_tuple(t.tuple(), PrimitiveType::Triangle));
    return (res / 2.) > m_val;
}

TodoSmallerInvariant::TodoSmallerInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& todo_handle,
    const double val)
    : Invariant(m, true, false, false)
    , m_todo_handle(todo_handle)
    , m_val(val)
{}

bool TodoSmallerInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == m_todo_handle.primitive_type());
    const attribute::Accessor<double> split_todo_accessor =
        mesh().create_const_accessor<double>(m_todo_handle);
    return split_todo_accessor.const_scalar_attribute(t.tuple()) < m_val;
}
} // namespace wmtk
