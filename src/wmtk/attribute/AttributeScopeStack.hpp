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
    AttributeScope<T>* current_scope_ptr();
    const AttributeScope<T>* current_scope_ptr() const;

    bool empty() const;
    void clear_current_scope();

    long depth() const;
    long add_checkpoint();
    AttributeScope<T> const* get_checkpoint(long index) const;

    void change_to_parent_scope() const;
    void change_to_leaf_scope() const;

    bool at_leaf_scope() const;
    bool writing_enabled() const;

    void flush_changes_to_vector(const Attribute<T>& attr, std::vector<T>& data) const;

protected:
    std::unique_ptr<AttributeScope<T>> m_leaf;
    mutable AttributeScope<T>* m_current = nullptr;
    std::vector<AttributeScope<T> const*> m_checkpoints;
    // Mesh& m_mesh;
    // AttributeManager& m_attribute_manager;
    // MeshAttributeHandle<T> m_handle;
};
} // namespace attribute
} // namespace wmtk
