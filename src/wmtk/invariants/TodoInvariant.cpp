#include "TodoInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk {
TodoInvariant::TodoInvariant(const Mesh& m, const MeshAttributeHandle<long>& todo_handle)
    : MeshInvariant(m)
    , m_todo_handle(todo_handle)
{}
bool TodoInvariant::before(const Simplex& t) const
{
    assert(t.primitive_type() == m_todo_handle.primitive_type());
    ConstAccessor<long> split_todo_accessor = mesh().create_accessor<long>(m_todo_handle);
    return split_todo_accessor.const_scalar_attribute(t.tuple()) == 1;
}
} // namespace wmtk
