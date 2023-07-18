#include "AttributeScopeHandle.hpp"
#include <spdlog/spdlog.h>
#include "AttributeManager.hpp"
#include "AttributeScope.hpp"
namespace wmtk {
AttributeScopeHandle::AttributeScopeHandle(AttributeManager& manager)
    : m_manager(manager)
{
    spdlog::info("creating the manager scope");
    m_manager.push_scope();
}


void AttributeScopeHandle::mark_failed()
{
    m_failed = true;
}
AttributeScopeHandle::~AttributeScopeHandle()
{
    spdlog::info("popping the manager scope");
    m_manager.pop_scope(!m_failed);
}
} // namespace wmtk
