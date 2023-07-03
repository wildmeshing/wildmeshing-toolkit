#include "Accessor.hpp"
#include "AccessorCache.hpp"

#include "Mesh.hpp"
#include "MeshAttributes.hpp"

namespace wmtk {
namespace {
constexpr static bool accessor_requires_caching(AccessorAccessMode mode)
{
    if (mode == AccessorAccessMode::Buffered) {
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
    AccessorAccessMode read_mode,
    AccessorAccessMdoe write_mode)
    : m_mesh(mesh)
    , m_handle(handle)
    , m_read_mode(read_mode)
    , m_write_mode(write_mode)
{
    if constexpr (IsConst) {
        assert(m_write_mode == AccessorAccessMode::None);
    }
    if (accessor_requires_caching(read_mode) || accessor_requires_caching(write_mode)) {
        m_cache = std::make_unique<AccessorCache<T>>(*this, read_mode, write_mode);
    }
}

template <typename T, bool IsConst>
Accessor<T, IsConst>::~Accessor()
{
    if (m_cache) {
        m_cache->flush(*this);
    }
}

template <typename T, bool IsConst>
AccessorAccessMode Accessor<T, IsConst>::read_access_mode() const
{
    return m_read_mode;
}

template <typename T, bool IsConst>
AccessorAccessMode Accessor<T, IsConst>::write_access_mode() const
{
    return m_write_mode;
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::vector_attribute(const long index) -> MapResultT
{
    auto buffer = attributes().vector_attribute(m_handle.m_base_handle, index);
    return buffer;
}

template <typename T, bool IsConst>
T Accessor<T, IsConst>::scalar_attribute(const long index) const
{
    auto value = attributes().scalar_attribute(m_handle.m_base_handle, index);
    return value;
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::scalar_attribute(const long index) -> TT
{
    auto& value = attributes().scalar_attribute(m_handle.m_base_handle, index);
}


template <typename T, bool IsConst>
auto Accessor<T, IsConst>::vector_attribute(const Tuple& t) const -> ConstMapResult
{
    long index = m_mesh.id(t, m_handle.m_primitive_type);
    return vector_attribute(index);
}
template <typename T, bool IsConst>
auto Accessor<T, IsConst>::vector_attribute(const Tuple& t) -> MapResultT
{
    long index = m_mesh.id(t, m_handle.m_primitive_type);
    return vector_attribute(index);
}

template <typename T, bool IsConst>
T Accessor<T, IsConst>::scalar_attribute(const Tuple& t) const
{
    long index = m_mesh.id(t, m_handle.m_primitive_type);
    return scalar_attribute(index);
}

template <typename T, bool IsConst>
auto Accessor<T, IsConst>::scalar_attribute(const Tuple& t) -> TT
{
    long index = m_mesh.id(t, m_handle.m_primitive_type);
    return scalar_attribute(index);
}

template <typename T, bool IsConst>
void Accessor<T, IsConst>::set_attribute(const std::vector<T>& value)
{
    if constexpr (IsConst) {
        throw std::runtime_error("You cant modify a constant accessor");
    } else
        attributes().set(m_handle.m_base_handle, value);
}

template <typename T, bool IsConst>
long Accessor<T, IsConst>::size() const
{
    return attributes().size();
}

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
