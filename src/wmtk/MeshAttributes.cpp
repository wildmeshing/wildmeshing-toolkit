#include "MeshAttributes.hpp"

#include <wmtk/utils/Rational.hpp>

#include <cassert>
#include <utility>

namespace wmtk {

template <typename T>
MeshAttributes<T>::MeshAttributes()
{}

template <typename T>
void MeshAttributes<T>::serialize(const int dim, MeshWriter& writer)
{
    for (const auto& p : m_handles) {
        const auto& handle = p.second;
        writer.write(p.first, dim, handle.stride, m_attributes[handle.index]);
    }
}

template <typename T>
AttributeHandle
MeshAttributes<T>::register_attribute(const std::string& name, long size, bool replace)
{
    assert(replace || m_handles.find(name) == m_handles.end());

    AttributeHandle handle;
    handle.stride = size;


    handle.index = m_attributes.size();
    m_handles[name] = handle;

    if (handle.index == 0) m_initial_stride = size;

    m_attributes.emplace_back();
    if (handle.index > 0) {
        assert(m_initial_stride > 0);
        assert(m_attributes.front().size() % m_initial_stride == 0);

        m_attributes.back().resize((m_attributes.front().size() / m_initial_stride) * size);
    } else if (m_internal_size > 0) {
        // first attribute and resize called
        m_attributes.front().resize(size * m_internal_size);
    }


    return handle;
}

template <typename T>
AttributeHandle MeshAttributes<T>::attribute_handle(const std::string& name) const
{
    return m_handles.at(name);
}

template <typename T>
bool MeshAttributes<T>::operator==(const MeshAttributes<T>& other) const
{
    return m_handles == other.m_handles && m_attributes == other.m_attributes;
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
void MeshAttributes<T>::set(const AttributeHandle& handle, const std::vector<T>& val)
{
    m_attributes[handle.index] = val;
}

template <typename T>
long MeshAttributes<T>::size() const
{
    const auto& attr = m_attributes;
    if (attr.empty()) return 0;

    return attr[0].size() / m_initial_stride;
}

template <typename T>
void MeshAttributes<T>::reserve(const long size)
{
    if (m_handles.empty()) m_internal_size = size;

    for (const auto& p : m_handles) {
        const auto& handle = p.second;
        m_attributes[handle.index].resize(handle.stride * size);
    }
}


template class MeshAttributes<char>;
template class MeshAttributes<long>;
template class MeshAttributes<double>;
template class MeshAttributes<Rational>;

} // namespace wmtk
