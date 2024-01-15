#pragma once
#include <memory>
#include <vector>
#include "AttributeHandle.hpp"

namespace wmtk {
class Mesh;
namespace attribute {
template <typename T>
class Attribute;
template <typename T>
class AccessorBase;
template <typename T>
class AttributeScope;
class AttributeManager;

/**
 * A stack of changes applied to an Attribute.
 * The stack consists of AttributeScopes which hold all changes applied inside one scope. Whenever a
 * new scope is created, it is pushed to the stack. As soon as an AttributeScopeHandle is
 * destructed, `push_scope` of all Attributes is triggered.
 */
template <typename T>
class AttributeScopeStack
{
public:
    // stack is implemented by a parent pointing graph, so we track a pointer
    // to the leaf
    AttributeScopeStack();
    ~AttributeScopeStack();
    void emplace();
    void pop(Attribute<T>& attribute, bool apply_updates);
#if !defined(WMTK_FLUSH_ON_FAIL)
    AttributeScope<T>* current_scope_ptr();
    const AttributeScope<T>* current_scope_ptr() const;
#endif

    bool empty() const;
    void clear_current_scope();

    int64_t depth() const;
#if defined(WMTK_ENABLE_GENERIC_CHECKPOINTS)
    int64_t add_checkpoint();
    AttributeScope<T> const* get_checkpoint(int64_t index) const;
#endif

    void change_to_parent_scope() const;
    void change_to_leaf_scope() const;

    bool at_leaf_scope() const;
    bool writing_enabled() const;

    void flush_changes_to_vector(const Attribute<T>& attr, std::vector<T>& data) const;
#if defined(WMTK_FLUSH_ON_FAIL)
    bool in_parent_scope() const { return m_in_parent_scope; }
#endif


protected:
    std::unique_ptr<AttributeScope<T>> m_leaf;
#if defined(WMTK_FLUSH_ON_FAIL)
    mutable bool m_in_parent_scope = false;
#else
    mutable AttributeScope<T>* m_current = nullptr;
#endif
#if defined(WMTK_ENABLE_GENERIC_CHECKPOINTS)
    std::vector<AttributeScope<T> const*> m_checkpoints;
#endif
    // Mesh& m_mesh;
    // AttributeManager& m_attribute_manager;
    // MeshAttributeHandle<T> m_handle;
};
} // namespace attribute
} // namespace wmtk
