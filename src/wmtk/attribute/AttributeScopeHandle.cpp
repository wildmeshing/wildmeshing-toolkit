#include "AttributeScopeHandle.hpp"
#include "AttributeManager.hpp"
#include "AttributeScope.hpp"
namespace wmtk::attribute {
AttributeScopeHandle::AttributeScopeHandle(AttributeManager& manager)
    : m_manager(manager)
{
    m_manager.push_scope();
}


void AttributeScopeHandle::mark_failed()
{
    m_failed = true;
    m_manager.clear_current_scope();
}
AttributeScopeHandle::~AttributeScopeHandle()
{
    m_manager.pop_scope(!m_failed);
}
} // namespace wmtk::attribute
