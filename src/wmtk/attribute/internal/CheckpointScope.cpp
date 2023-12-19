#include "CheckpointScope.hpp"

#include <wmtk/attribute/AttributeManager.hpp>
namespace wmtk::attribute::internal {
CheckpointScope::CheckpointScope(wmtk::attribute::AttributeManager& manager)
    : m_manager(manager)
{
    m_manager.change_to_parent_scope();
}
CheckpointScope::~CheckpointScope()
{
    m_manager.change_to_leaf_scope();
}
} // namespace wmtk::attribute::internal
