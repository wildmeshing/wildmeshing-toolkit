#pragma once
#include <memory>
#include <vector>
#include "AttributeCache.hpp"
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
    using MapResult = typename AttributeCache<T>::MapResult;
    using ConstMapResult = typename AttributeCache<T>::ConstMapResult;
    // stack is implemented by a parent pointing graph, so we track a pointer
    // to the leaf
    AttributeScopeStack();
    ~AttributeScopeStack();
    AttributeScopeStack(const AttributeScopeStack&) = delete;
    AttributeScopeStack& operator=(const AttributeScopeStack&) = delete;
    void emplace();
    void pop(Attribute<T>& attribute, bool apply_updates);
    AttributeScope<T>* active_scope_ptr();
    const AttributeScope<T>* active_scope_ptr() const;

    bool empty() const;
    void clear_current_scope(Attribute<T>& attr);

    int64_t depth() const;
#if defined(WMTK_ENABLE_GENERIC_CHECKPOINTS)
    int64_t add_checkpoint();
    AttributeScope<T> const* get_checkpoint(int64_t index) const;
#endif

    // go to the next most historic scope
    void change_to_next_scope() const;
    void change_to_previous_scope() const;
    // go to the scope with active data
    void change_to_current_scope() const;

    bool at_current_scope() const;
    bool writing_enabled() const;

    void flush_changes_to_vector(const Attribute<T>& attr, std::vector<T>& data) const;

#if defined(WMTK_FLUSH_ON_FAIL)
    MapResult vector_attribute(AccessorBase<T>& accessor, int64_t index);

    ConstMapResult const_vector_attribute(const AccessorBase<T>& accessor, int64_t index) const;


    T& scalar_attribute(AccessorBase<T>& accessor, int64_t index);

    T const_scalar_attribute(const AccessorBase<T>& accessor, int64_t index) const;
#endif

protected:
    std::unique_ptr<AttributeScope<T>> m_start;
    mutable AttributeScope<T>* m_active = nullptr;
#if defined(WMTK_ENABLE_GENERIC_CHECKPOINTS)
    std::vector<AttributeScope<T> const*> m_checkpoints;
#endif
    // Mesh& m_mesh;
    // AttributeManager& m_attribute_manager;
    // MeshAttributeHandle<T> m_handle;
};
} // namespace attribute
} // namespace wmtk
