#include "AttributeScopeManager.hpp"
#include "AttributeScope.hpp"

namespace wmtk {


AttributeScopeHandle AttributeScopeManager::create_scope(Mesh& m)
{
    m_stacks.local().emplace(mesh);

    return AttributeScopeHandle(*this);
}

AttributeScopeManager::AttributeScopeManager() = default;

AttributeScope* AttributeScopeManager::get_scope(Mesh& m)
{
    auto& local_stack = m_stacks.local();
    auto& leaf = local_stack.m_leaf;
    // TODO:can we safely get if the internal content is empty?
    if (bool(leaf)) {
        return leaf.get();
    } else {
        return nullptr;
    }
}
} // namespace wmtk
