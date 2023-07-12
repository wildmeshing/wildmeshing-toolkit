#include "Accessor.hpp"
#include "attribute/AttributeCache.hpp"

#include "Mesh.hpp"
#include "MeshAttributes.hpp"

namespace wmtk {
namespace {
constexpr static bool accessor_requires_caching(AttributeAccessMode mode)
{
    if (mode == AttributeAccessMode::Buffered) {
        return true;
    } else {
        return false;
    }
}
} // namespace

template <typename T, bool IsConst>
Accessor<T, IsConst>::Accessor(
    MeshType& mesh,
    const MeshAttributeHandle<T>& handle,
    AttributeAccessMode mode)
    : BaseType(const_cast<Mesh&>(mesh), handle)
    , m_mode(mode)
{
    if (accessor_requires_caching(mode)) {
        m_cache = mesh.request_accessor_cache();
    }
}

template <typename T, bool IsConst>
Accessor<T, IsConst>::~Accessor()
{
    // if constexpr (!IsConst) {
    //     if (m_cache) {
    //         m_cache->flush(*this);
    //     }
    // }
}

template <typename T, bool IsConst>
AttributeAccessMode Accessor<T, IsConst>::access_mode() const
{
    return m_mode;
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::cacheable_const_vector_attribute(const long index) const
    -> ConstMapResult
{
    if (m_cache) {
        return m_cache->const_vector_attribute(*this, m_mode, index);
    } else {
        return BaseType::const_vector_attribute(index);
    }
}


template <typename T, bool IsConst>
auto Accessor<T, IsConst>::cacheable_vector_attribute(const long index) -> MapResultT
{
    if (m_cache) {
        return m_cache->vector_attribute(*this, m_mode, index);
    } else {
        return BaseType::vector_attribute(index);
    }
}

template <typename T, bool IsConst>
T Accessor<T, IsConst>::cacheable_const_scalar_attribute(const long index) const
{
    if (m_cache) {
        return m_cache->const_scalar_attribute(*this, m_mode, index);
    } else {
        return BaseType::const_scalar_attribute(index);
    }
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::cacheable_scalar_attribute(const long index) -> TT
{
    if (m_cache) {
        return m_cache->scalar_attribute(*this, m_mode, index);
    } else {
        return BaseType::scalar_attribute(index);
    }
}


//===================================================
// These following methods just forward to to const names
template <typename T, bool IsConst>
auto Accessor<T, IsConst>::vector_attribute(const Tuple& t) const -> ConstMapResult
{
    return const_vector_attribute(t);
}
template <typename T, bool IsConst>
T Accessor<T, IsConst>::scalar_attribute(const Tuple& t) const
{
    return const_scalar_attribute(t);
}
//===================================================


//===================================================
// These methods just compute the index and forward it
template <typename T, bool IsConst>
auto Accessor<T, IsConst>::const_vector_attribute(const Tuple& t) const -> ConstMapResult
{
    const long idx = BaseType::index(t);
    return cacheable_const_vector_attribute(idx);
}
template <typename T, bool IsConst>
auto Accessor<T, IsConst>::vector_attribute(const Tuple& t) -> MapResultT
{
    const long idx = BaseType::index(t);
    return cacheable_vector_attribute(idx);
}


template <typename T, bool IsConst>
T Accessor<T, IsConst>::const_scalar_attribute(const Tuple& t) const
{
    const long idx = BaseType::index(t);
    return cacheable_const_scalar_attribute(idx);
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::scalar_attribute(const Tuple& t) -> TT
{
    const long idx = BaseType::index(t);
    return cacheable_scalar_attribute(idx);
}
//===================================================


// template <typename T, bool IsConst>
// void MeshAttributes<T>::rollback()
//{
//     attributes()s_copy.clear();
// }
//
// template <typename T, bool IsConst>
// void MeshAttributes<T>::begin_protect()
//{
//     attributes()s_copy =
//     attributes()s;
// }
//
// template <typename T, bool IsConst>
// void MeshAttributes<T>::end_protect()
//{
//     if (!attributes()s_copy.empty())
//     attributes()s =
//     std::move(attributes()s_copy);
//
//     attributes()s_copy.clear();
// }
//
// template <typename T, bool IsConst>
// bool MeshAttributes<T>::is_in_protect() const
//{
//     return !attributes()s_copy.empty();
// }
template class Accessor<char, true>;
template class Accessor<long, true>;
template class Accessor<double, true>;
template class Accessor<char, false>;
template class Accessor<long, false>;
template class Accessor<double, false>;
} // namespace wmtk
