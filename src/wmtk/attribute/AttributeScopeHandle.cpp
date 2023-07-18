#include "AttributeScopeHandle.hpp"
#include "AttributeScope.hpp"
#include "AttributeManager.hpp"
namespace wmtk {
AttributeScopeHandle::AttributeScopeHandle(AttributeManager& manager): m_manager(manager)
{
    m_manager.push_scope();
}


void AttributeScopeHandle::mark_failed() {
    m_failed = true;
}
AttributeScopeHandle::~AttributeScopeHandle()
{
    m_manager.pop_scope(!m_failed);
}
} // namespace wmtk
