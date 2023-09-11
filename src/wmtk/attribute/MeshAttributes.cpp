#include "MeshAttributes.hpp"
#include "PerThreadAttributeScopeStacks.hpp"

#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/utils/Rational.hpp>

#include <cassert>
#include <utility>

namespace wmtk::attribute {

template <typename T>
MeshAttributes<T>::MeshAttributes()
{}
template <typename T>
MeshAttributes<T>::MeshAttributes(const MeshAttributes& o) = default;
template <typename T>
MeshAttributes<T>::MeshAttributes(MeshAttributes&& o) = default;
template <typename T>
MeshAttributes<T>& MeshAttributes<T>::operator=(const MeshAttributes& o) = default;
template <typename T>
MeshAttributes<T>& MeshAttributes<T>::operator=(MeshAttributes&& o) = default;

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
void MeshAttributes<T>::push_scope()
{
    for (auto& attr : m_attributes) {
        attr.push_scope();
    }
}
template <typename T>
void MeshAttributes<T>::pop_scope(bool apply_updates)
{
    for (auto& attr : m_attributes) {
        attr.pop_scope(apply_updates);
    }
}
template <typename T>
void MeshAttributes<T>::clear_current_scope()
{
    for (auto& attr : m_attributes) {
        attr.clear_current_scope();
    }
}
template <typename T>
AttributeHandle
MeshAttributes<T>::register_attribute(const std::string& name, long dimension, bool replace)
{
    assert(replace || m_handles.find(name) == m_handles.end());

    AttributeHandle handle;


    if (replace && m_handles.find(name) != m_handles.end()) {
        auto it = m_handles.find(name);
        handle.index = it->second.index;
    } else {
        handle.index = m_attributes.size();
        m_attributes.emplace_back(dimension, reserved_size());
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
Attribute<T>& MeshAttributes<T>::attribute(const AttributeHandle& handle)
{
    Attribute<T>& attr = m_attributes[handle.index];
    return attr;
}
template <typename T>
const Attribute<T>& MeshAttributes<T>::attribute(const AttributeHandle& handle) const
{
    return m_attributes[handle.index];
}


template <typename T>
void MeshAttributes<T>::set(const AttributeHandle& handle, std::vector<T> val)
{
    // TODO: should we validate the size of val compared to the internally held data?
    auto& attr = m_attributes[handle.index];
    attr.set(std::move(val));
}

template <typename T>
size_t MeshAttributes<T>::attribute_size(const AttributeHandle& handle) const
{
    return m_attributes[handle.index].reserved_size();
}

template <typename T>
long MeshAttributes<T>::reserved_size() const
{
    return m_reserved_size;
}

template <typename T>
void MeshAttributes<T>::reserve(const long size)
{
    m_reserved_size = size;
    for (auto& attr : m_attributes) {
        attr.reserve(size);
    }
}


template class MeshAttributes<char>;
template class MeshAttributes<long>;
template class MeshAttributes<double>;
template class MeshAttributes<Rational>;

} // namespace wmtk::attribute
