#pragma once
#include <memory>
#include <wmtk/AttributeHandle.hpp>

namespace wmtk {
class Mesh;
template <typename T>
class Attribute;
template <typename T>
class AccessorBase;
template <typename T>
class AttributeScope;
class AttributeManager;


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


protected:
    std::unique_ptr<AttributeScope<T>> m_leaf;
    // Mesh& m_mesh;
    // AttributeManager& m_attribute_manager;
    // MeshAttributeHandle<T> m_handle;
};
} // namespace wmtk
