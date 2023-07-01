#include "MeshAttributes.hpp"

#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/utils/Rational.hpp>

#include <cassert>
#include <utility>

namespace wmtk {

template <typename T>
MeshAttributes<T>::MeshAttributes()
{}

template <typename T>
void MeshAttributes<T>::serialize(const int dim, MeshWriter& writer) const
{
    for (const auto& p : m_handles) {
        const auto& handle = p.second;
        const auto& attr = m_attributes[handle.index];
        attr.serialize(p.first, dim, writer);
    }
}

template <typename T>
AttributeHandle
MeshAttributes<T>::register_attribute(const std::string& name, long stride, bool replace)
{
    assert(replace || m_handles.find(name) == m_handles.end());

    AttributeHandle handle;


    if (replace && m_handles.find(name) != m_handles.end()) {
        auto it = m_handles.find(name);
        handle.index = it->second.index;
    } else {
        handle.index = m_attributes.size();
        m_attributes.emplace_back(stride, size());
    }
    m_handles[name] = handle;


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
    return attr.vector_attribute(index);
}


template <typename T>
typename MeshAttributes<T>::MapResult MeshAttributes<T>::vector_attribute(
    const AttributeHandle& handle,
    const long index)
{
    auto& attr = m_attributes[handle.index];
    return attr.vector_attribute(index);
}

template <typename T>
T MeshAttributes<T>::scalar_attribute(const AttributeHandle& handle, const long index) const
{
    const auto& attr = m_attributes[handle.index];
    return attr.scalar_attribute(index);
}

template <typename T>
T& MeshAttributes<T>::scalar_attribute(const AttributeHandle& handle, const long index)
{
    auto& attr = m_attributes[handle.index];
    return attr.scalar_attribute(index);
}

template <typename T>
void MeshAttributes<T>::set(const AttributeHandle& handle, std::vector<T> val)
{
    // TODO: should we validate the size of val compared to the internally held data?
    auto& attr = m_attributes[handle.index];
    attr.set(std::move(val));
}

template <typename T>
long MeshAttributes<T>::size() const
{
    return m_size;
}

template <typename T>
void MeshAttributes<T>::reserve(const long size)
{
    m_size = size;
    for (auto& attr : m_attributes) {
        attr.reserve(size);
    }
}


template class MeshAttributes<char>;
template class MeshAttributes<long>;
template class MeshAttributes<double>;
template class MeshAttributes<Rational>;

} // namespace wmtk
