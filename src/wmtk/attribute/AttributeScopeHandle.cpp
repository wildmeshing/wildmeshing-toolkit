#include "AttributeScopeHandle.hpp"
#include "AttributeManager.hpp"
#include "AttributeScope.hpp"
namespace wmtk::attribute {
AttributeScopeHandle::AttributeScopeHandle(AttributeManager& manager)
    : m_manager(manager)
{
    m_manager.push_scope();
}


AttributeScopeHandle::AttributeScopeHandle(AttributeScopeHandle&& o)
    : m_manager(o.m_manager)
    , m_failed(o.m_failed)
    , m_was_moved(o.m_was_moved)
{
    o.m_was_moved = true;
}

void AttributeScopeHandle::mark_failed()
{
    // the dev should know if they moved a handle and then tried to mark it as failed
    assert(!m_was_moved);
    if (!m_was_moved) {
        m_failed = true;
        m_manager.rollback_current_scope();
    }
}
AttributeScopeHandle::~AttributeScopeHandle()
{
    if (!m_was_moved) {
        m_manager.pop_scope(!m_failed);
    }
}
} // namespace wmtk::attribute
