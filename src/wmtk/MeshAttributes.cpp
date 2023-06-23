#pragma once
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
AttributeHandle MeshAttributes<T>::attribute_handle(const std::string& name) const
{
    return m_handles.at(name);
}


template <typename T>
auto MeshAttributes<T>::vector_attribute(const AttributeHandle& handle, const long index) const
    -> ConstMapResult
{
    const auto& attr = m_attributes[handle.index];
    const long start = index * handle.stride;

    return ConstMapResult(&attr[start], handle.stride);
}


template <typename T>
typename MeshAttributes<T>::MapResult MeshAttributes<T>::vector_attribute(
    const AttributeHandle& handle,
    const long index)
{
    auto& attr = m_attributes[handle.index];
    const long start = index * handle.stride;

    return MapResult(&attr[start], handle.stride);
}

template <typename T>
T MeshAttributes<T>::scalar_attribute(const AttributeHandle& handle, const long index) const
{
    assert(handle.stride == 1);
    const auto& attr = m_attributes[handle.index];

    return attr[index];
}

template <typename T>
T& MeshAttributes<T>::scalar_attribute(const AttributeHandle& handle, const long index)
{
    assert(handle.stride == 1);
    auto& attr = m_attributes[handle.index];

    return attr[index];
}

template <typename T>
long MeshAttributes<T>::size() const
{
    const auto& attr = m_attributes;
    if (attr.empty()) return 0;

    return attr[0].size() / initial_stride;
}

template <typename T>
void MeshAttributes<T>::reserve(const long size)
{
    auto& attr = m_attributes;

    for (const auto& p : m_handles) {
        const auto& handle = p.second;
        attr[handle.index].resize(handle.stride * size);
    }
}


template class MeshAttributes<char>;
template class MeshAttributes<long>;
template class MeshAttributes<double>;
// template class MeshAttributes<bool>;

} // namespace wmtk
