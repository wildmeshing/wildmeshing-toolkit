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
    AttributeScope(std::unique_ptr<AttributeScope<T>>&& next);


private:
    using MapResult = typename AttributeCache<T>::MapResult;
    using ConstMapResult = typename AttributeCache<T>::ConstMapResult;
    using DataStorage = typename AttributeCache<T>::DataStorage;
    using AttributeCache<T>::m_data;

#if !defined(WMTK_FLUSH_ON_FAIL)
    MapResult load_cached_vector_value(AccessorBase<T>& accessor, int64_t index);
    T& load_cached_scalar_value(AccessorBase<T>& accessor, int64_t index);
#endif
    ConstMapResult load_const_cached_vector_value(const AccessorBase<T>& accessor, int64_t index)
        const;
    T load_const_cached_scalar_value(const AccessorBase<T>& accessor, int64_t index) const;

    AttributeCache<T>& get_cache() { return static_cast<AttributeCache<T>&>(*this); }

    // flushes cache to previous scope unless there is no previous scope, in which
    // case it flushes data to the underlying attribute storage
    void flush(Attribute<T>& attr);

    void flush_changes_to_vector(const Attribute<T>& attr, std::vector<T>& data);

    std::unique_ptr<AttributeScope<T>> pop_to_next();
#if !defined(WMTK_FLUSH_ON_FAIL)
    MapResult vector_attribute(AccessorBase<T>& accessor, int64_t index);
    T& scalar_attribute(AccessorBase<T>& accessor, int64_t index);
#endif

    ConstMapResult const_vector_attribute(const AccessorBase<T>& accessor, int64_t index) const;


    T const_scalar_attribute(const AccessorBase<T>& accessor, int64_t index) const;

    int64_t depth() const;

#if defined(WMTK_ENABLE_GENERIC_CHECKPOINTS)
    int64_t checkpoint_index() const { return m_checkpoint_index; }
#endif

    const AttributeScope<T>* previous() const { return m_previous; }
    AttributeScope<T>* previous() { return m_previous; }

    const AttributeScope<T>* next() const { return m_next.get(); }
    AttributeScope<T>* next() { return m_next.get(); }

private:
    std::unique_ptr<AttributeScope<T>> m_next = nullptr;
    //
    AttributeScope<T>* m_previous = nullptr;
#if defined(WMTK_ENABLE_GENERIC_CHECKPOINTS)
    int64_t m_checkpoint_index = -1;
#endif
};

} // namespace attribute
} // namespace wmtk
#include "AttributeScope.hxx"
