#pragma once
#include <tbb/enumerable_thread_specific.h>
namespace wmtk {
class Mesh;
class AttributeScope;


// Maintains per-thread stacks of attribute scopes
// Should only be constructed and manipulated by the mesh class
class AttributeScopeManager
{
    friend class Mesh;
    AttributeScopeManager();
    // returns nullptr if the scope does not exist
    AttributeScope* get_scope();

    struct AttributeScopeStack;
    tbb::enumerable_thread_specific<AttributeScopeStack> m_stacks;
};

struct AttributeScopeManager::AttributeScopeStack
{
    // stack is implemented by a parent pointing graph, so we track a pointer
    // to the leaf
    AttributeScopeStack();
    ~AttributeScopeStack();
    AttributeScope& emplace(Mesh& mesh);
    void pop();

    std::unique_ptr<AttributeScope> m_leaf;
};
} // namespace wmtk
