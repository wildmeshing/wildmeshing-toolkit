#pragma once

#include <memory>
#include "Attribute.hpp"
#include "AttributeAccessMode.hpp"
#include "AttributeCache.hpp"

namespace wmtk {


namespace attribute {
template <typename T>
class AttributeScopeStack;
template <typename T>
class CachingAccessor;

template <typename T>
class AttributeScope : public AttributeCache<T>
{
public:
    friend class CachingAccessor<T>;
    friend class AttributeScopeStack<T>;
    AttributeScope();
    AttributeScope(const AttributeScope&) = delete;
    AttributeScope& operator=(const AttributeScope&) = delete;
    ~AttributeScope();
    AttributeScope(std::unique_ptr<AttributeScope<T>>&& parent);


private:
    using MapResult = typename AttributeCache<T>::MapResult;
    using ConstMapResult = typename AttributeCache<T>::ConstMapResult;
    using DataStorage = typename AttributeCache<T>::DataStorage;
    using AttributeCache<T>::m_data;
    MapResult load_cached_vector_value(AccessorBase<T>& accessor, int64_t index);
    ConstMapResult load_const_cached_vector_value(const AccessorBase<T>& accessor, int64_t index)
        const;
    T& load_cached_scalar_value(AccessorBase<T>& accessor, int64_t index);
    T load_const_cached_scalar_value(const AccessorBase<T>& accessor, int64_t index) const;
    // returns an iterator and makes sure a value is set
    typename DataStorage::iterator load_it(
        const AccessorBase<T>& accessor,
        AttributeAccessMode mode,
        int64_t index,
        bool mark_dirty = false) const;

    AttributeCache<T>& get_cache() { return static_cast<AttributeCache<T>&>(*this); }

    // flushes cache to parent scope unless there is no parent scope, in which
    // case it flushes data to the underlying attribute storage
    void flush(Attribute<T>& attr);

    void flush_changes_to_vector(const Attribute<T>& attr, std::vector<T>& data);

    // pops the parent scope
    std::unique_ptr<AttributeScope<T>> pop_parent();
    MapResult vector_attribute(AccessorBase<T>& accessor, AttributeAccessMode mode, int64_t index);

    ConstMapResult const_vector_attribute(
        const AccessorBase<T>& accessor,
        AttributeAccessMode mode,
        int64_t index) const;


    T& scalar_attribute(AccessorBase<T>& accessor, AttributeAccessMode mode, int64_t index);

    T const_scalar_attribute(
        const AccessorBase<T>& accessor,
        AttributeAccessMode mode,
        int64_t index) const;

    int64_t depth() const;

    int64_t checkpoint_index() const { return m_checkpoint_index; }

    const AttributeScope<T>* parent() const { return m_parent.get(); }
    AttributeScope<T>* parent() { return m_parent.get(); }

private:
    std::unique_ptr<AttributeScope<T>> m_parent;
    int64_t m_checkpoint_index = -1;
};

} // namespace attribute
} // namespace wmtk
