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
    AttributeScope(AttributeScope&&) = default;
    AttributeScope& operator=(AttributeScope&&) = default;
    ~AttributeScope();
    AttributeScope(std::unique_ptr<AttributeScope<T>>&& next);

    using AttributeCache<T>::size;
    // flushes cache to previous scope unless there is no previous scope, in which
    // case it flushes data to the underlying attribute storage
    void apply(Attribute<T>& attr) const;

    void apply(const Attribute<T>& attr, std::vector<T>& data);

private:
    using MapResult = typename AttributeCache<T>::MapResult;
    using ConstMapResult = typename AttributeCache<T>::ConstMapResult;
    using DataStorage = typename AttributeCache<T>::DataStorage;
    using AttributeCache<T>::m_data;

    ConstMapResult load_const_cached_vector_value(const AccessorBase<T>& accessor, int64_t index)
        const;
    T load_const_cached_scalar_value(const AccessorBase<T>& accessor, int64_t index) const;

    AttributeCache<T>& get_cache() { return static_cast<AttributeCache<T>&>(*this); }
};

} // namespace attribute
} // namespace wmtk
#include "AttributeScope.hxx"
