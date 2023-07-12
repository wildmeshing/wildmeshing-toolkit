#pragma once

#include <memory>

namespace wmtk {


namespace detail {

template <typename T>
class AttributeAccessorCache
{
    std::shared_ptr<AttributeCache<T>> get_cache(const AttributeHandle& handle);
    std::vector<std::shared_ptr<AttributeCache<T>>> m_caches;
};

template <typename U>
typename CachePtrType = std::shared_ptr<AttributeAccessorCache<U>>;
template <typename... T>
using CachePtrStorage = std::tuple<CachePtrType<T>...>;

} // namespace detail
class AttributeScope
{
public:
    AttributeScope();
    AttributeScope(std::unique_ptr<AttributeScope>&& parent);
    AttributeScope(Mesh& mesh);


    ~AttributeScope();

    template <typename U>
    CachePtrType<U> get_caches_single_type(PrimitiveType type)
    {
        return std::get<U>(m_caches_per_type[simplex_dimension(type)]);
    }
    template <typename U>
    std::shared_ptr<AttributeCache<T>> get_cache(const MeshAttributeHandle<T>& handle)
    {
        return get_caches_single_type<T>(handle.m_primitive_type).get_cache(handle.m_handle);
    }

    // pops the parent scope
    std::unique_ptr<AttributeScope> pop_parent() { return std::move(m_parent); }

private:
    // stores a shared_ptr<AttributeAccessorCache<T>> for each type we awnt
    // fr each dimension
    std::vector<CachePtrStorage<double, long, char>> m_caches_per_dimension_per_type;
    std::unique_ptr<AttributeScope> m_parent;
};

template <typename U>
CachePtrType<U> get_caches_single_type(PrimitiveType type)
{
    return std::get<U>(m_caches_per_type[simplex_dimension(type)]);
}
template <typename U>
std::shared_ptr<AttributeCache<T>> get_cache(const MeshAttributeHandle<T>& handle)
{
    return get_caches_single_type<T>(handle.m_primitive_type).get_cache(handle.m_handle);
}
} // namespace wmtk
