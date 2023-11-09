#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "MeshInvariant.hpp"

namespace wmtk {
class TodoInvariant : public MeshInvariant
{
    /**
     * Invariant for todo-list in scheduler. Recording which simplicity still need to be operated.
     * If the todo_tag tagged as 1 then return true, otherwise return false
     */
public:
    TodoInvariant(const Mesh& m, const MeshAttributeHandle<long>& todo_handle);
    bool before(const Tuple& t) const override;

private:
    const MeshAttributeHandle<long> m_todo_handle;
};
} // namespace wmtk