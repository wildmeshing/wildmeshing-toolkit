#include "AttributeScopeHandle.hpp"
#include "AttributeScope.hpp"
#include "AttributeManager.hpp"
namespace wmtk {
AttributeScopeHandle::AttributeScopeHandle(AttributeManager& manager): m_manager(manager)
{
    m_manager.push_scope();
}
AttributeScopeHandle::~AttributeScopeHandle()
{
    m_manager.pop_scope();
}
} // namespace wmtk
