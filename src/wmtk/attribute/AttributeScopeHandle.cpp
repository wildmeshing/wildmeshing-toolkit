#include "AttributeScopeHandle.hpp"
#include "AttributeScopeManager.hpp"
#include "AttributeScope.hpp"
namespace wmtk {
    AttributeScopeHandle::AttributeScopeHandle(AttributeScopeManager& scope) {
        m_scope_manager.push_scope();
    }
    AttributeScopeHandle::~AttributeScopeHandle() {
        m_scope_manager.pop_scope();
    }
}
