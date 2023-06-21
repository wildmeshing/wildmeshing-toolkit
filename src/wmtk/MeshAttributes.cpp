#include "MeshAttributes.hpp"

#include <cassert>
#include <utility>

namespace wmtk {

template <typename T>
MeshAttributes<T>::MeshAttributes()
{}

template <typename T>
AttributeHandle MeshAttributes<T>::register_attribute(const std::string& name, long size)
{
    assert(m_handles.find(name) == m_handles.end());
    assert(!is_in_protect());

    AttributeHandle handle;
    handle.stride = size;


    handle.index = m_attributes.size();
    m_handles[name] = handle;

    if (handle.index == 0) initial_stride = size;

    m_attributes.emplace_back();
    if (handle.index > 0) {
        assert(initial_stride > 0);
        assert(m_attributes.front().size() % initial_stride == 0);

        m_attributes.back().resize((m_attributes.front().size() / initial_stride) * size);
    }


    return handle;
}

template <typename T>
AttributeHandle MeshAttributes<T>::get_attribute_handle(const std::string& name) const
{
    return m_handles.at(name);
}

template <typename T>
const typename MeshAttributes<T>::ConstMapResult MeshAttributes<T>::get_vector_attribute(
    const std::string& name,
    const long index) const
{
    const auto& handle = m_handles.at(name);
    return get_vector_attribute(handle, index);
}

template <typename T>
const typename MeshAttributes<T>::ConstMapResult MeshAttributes<T>::get_vector_attribute(
    const AttributeHandle& handle,
    const long index) const
{
    const auto& attr = current_attributes()[handle.index];
    const long start = index * handle.stride;

    return ConstMapResult(&attr[start], handle.stride);
}

template <typename T>
typename MeshAttributes<T>::MapResult MeshAttributes<T>::get_vector_attribute(
    const std::string& name,
    const long index)
{
    const auto& handle = m_handles.at(name);
    return get_vector_attribute(handle, index);
}

template <typename T>
typename MeshAttributes<T>::MapResult MeshAttributes<T>::get_vector_attribute(
    const AttributeHandle& handle,
    const long index)
{
    auto& attr = current_attributes()[handle.index];
    const long start = index * handle.stride;

    return MapResult(&attr[start], handle.stride);
}

template <typename T>
T MeshAttributes<T>::get_scalar_attribute(const std::string& name, const long index) const
{
    const auto& handle = m_handles.at(name);
    return get_scalar_attribute(handle, index);
}

template <typename T>
T MeshAttributes<T>::get_scalar_attribute(const AttributeHandle& handle, const long index) const
{
    assert(handle.stride == 1);
    const auto& attr = current_attributes()[handle.index];

    return attr[index];
}

template <typename T>
T& MeshAttributes<T>::get_scalar_attribute(const std::string& name, const long index)
{
    const auto& handle = m_handles.at(name);
    return get_scalar_attribute(handle, index);
}

template <typename T>
T& MeshAttributes<T>::get_scalar_attribute(const AttributeHandle& handle, const long index)
{
    assert(handle.stride == 1);
    auto& attr = current_attributes()[handle.index];

    return attr[index];
}

template <typename T>
long MeshAttributes<T>::size() const
{
    const auto& attr = current_attributes();
    if (attr.empty()) return 0;

    return attr[0].size() / initial_stride;
}

template <typename T>
void MeshAttributes<T>::resize(const long size)
{
    auto& attr = current_attributes();

    for (const auto& p : m_handles) {
        const auto& handle = p.second;
        attr[handle.index].resize(handle.stride * size);
    }
}

template <typename T>
void MeshAttributes<T>::rollback()
{
    m_attributes_copy.clear();
}

template <typename T>
void MeshAttributes<T>::begin_protect()
{
    m_attributes_copy = m_attributes;
}

template <typename T>
void MeshAttributes<T>::end_protect()
{
    if (!m_attributes_copy.empty()) m_attributes = std::move(m_attributes_copy);

    m_attributes_copy.clear();
}

template <typename T>
bool MeshAttributes<T>::is_in_protect() const
{
    return !m_attributes_copy.empty();
}

// template class MeshAttributes<bool>;
template class MeshAttributes<long>;
template class MeshAttributes<double>;
// template class MeshAttributes<bool>;

} // namespace wmtk
