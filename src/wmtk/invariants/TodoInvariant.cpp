#include "TodoInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk {
TodoInvariant::TodoInvariant(const Mesh& m, const MeshAttributeHandle<long>& todo_handle)
    : MeshInvariant(m)
    , m_todo_handle(todo_handle)
{}
bool TodoInvariant::before(const Tuple& t) const
{
    ConstAccessor<long> split_todo_accessor = mesh().create_accessor<long>(m_todo_handle);
    return split_todo_accessor.const_scalar_attribute(t) == 1;
}
} // namespace wmtk