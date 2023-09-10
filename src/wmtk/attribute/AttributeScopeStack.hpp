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
struct AttributeManager;


template <typename T>
struct AttributeScopeStack
{
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


protected:
    std::unique_ptr<AttributeScope<T>> m_leaf;
    std::vector<AttributeScope<T> const*> m_checkpoints;
    // Mesh& m_mesh;
    // AttributeManager& m_attribute_manager;
    // MeshAttributeHandle<T> m_handle;
};
} // namespace attribute
} // namespace wmtk
