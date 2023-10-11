#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "MeshInvariant.hpp"

namespace wmtk {
class TodoInvariant : public MeshInvariant
{
public:
    // NOTE: this takes in the threshold squared rather than the threshold itself
    TodoInvariant(const Mesh& m, const MeshAttributeHandle<long>& todo_handle);
    bool before(const Tuple& t) const override;

private:
    const MeshAttributeHandle<long> m_todo_handle;
};
} // namespace wmtk