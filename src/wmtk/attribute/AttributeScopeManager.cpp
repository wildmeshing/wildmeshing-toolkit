#include "AttributeScopeManager.hpp"
#include "AttributeScope.hpp"

namespace wmtk {

AttributeScopeStack::AttributeScopeStack() = default;
AttributeScopeStack::~AttributeScopeStack() = default;
AttributeScope& AttributeScopeStack::emplace(Mesh& m)
{
    // create a new leaf that points to the current stack and
    //
    std::unique_ptr<AttributeScope> new_leaf(new AttributeScope(std::move(m_leaf)));
    m_leaf = std::move(new_leaf);
    return *m_leaf;
}
void AttributeScopeStack::pop()
{
    // delete myself by setting my child to be the leaf
    assert(bool(m_leaf));
    m_leaf = std::move(m_leaf.pop_child());
}

AttributeScopeManager::AttributeScopeManager() = default;

AttributeScope* AttributeScopeManager::get_scope()
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
