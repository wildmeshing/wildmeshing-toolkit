#include "MeshAttributes.hpp"
#include <wmtk/attribute/internal/hash.hpp>
#include <wmtk/utils/Hashable.hpp>
#include "PerThreadAttributeScopeStacks.hpp"

#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/utils/Rational.hpp>

#include <cassert>
#include <functional>
#include <stdexcept>
#include <string>
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
std::map<std::string, std::size_t> MeshAttributes<T>::child_hashes() const
{
    // default implementation pulls the child attributes (ie the attributes)
    std::map<std::string, std::size_t> ret = wmtk::utils::MerkleTreeInteriorNode::child_hashes();

    // hash handle data
    for (const auto& [name, handle] : m_handles) {
        ret["attr_handle_" + name] = std::hash<AttributeHandle>{}(handle);
    }
    return ret;
}
template <typename T>
std::map<std::string, const wmtk::utils::Hashable*> MeshAttributes<T>::child_hashables() const

{
    std::map<std::string, const wmtk::utils::Hashable*> ret;
    for (const auto& [name, handle] : m_handles) {
        const auto& attr = attribute(handle);
        ret["attr_" + name] = &attr;
    }
    return ret;
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
void MeshAttributes<T>::change_to_parent_scope() const
{
    for (const auto& attr : m_attributes) {
        attr.get_local_scope_stack_ptr()->change_to_parent_scope();
    }
}

template <typename T>
void MeshAttributes<T>::change_to_leaf_scope() const
{
    for (const auto& attr : m_attributes) {
        attr.get_local_scope_stack_ptr()->change_to_leaf_scope();
    }
}

template <typename T>
AttributeHandle MeshAttributes<T>::register_attribute(
    const std::string& name,
    long dimension,
    bool replace,
    T default_value)
{
    assert(replace || m_handles.find(name) == m_handles.end());

    AttributeHandle handle;


    if (replace && m_handles.find(name) != m_handles.end()) {
        auto it = m_handles.find(name);
        handle.index = it->second.index;
    } else {
        handle.index = m_attributes.size();
        m_attributes.emplace_back(dimension, default_value, reserved_size());
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
bool MeshAttributes<T>::has_attribute(const std::string& name) const
{
    return m_handles.find(name) != m_handles.end();
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

template <typename T>
void MeshAttributes<T>::reserve_more(const long size)
{
    reserve(m_reserved_size + size);
}
template <typename T>
void MeshAttributes<T>::clear_attributes(const std::vector<AttributeHandle>& keep_attributes)
{
    std::vector<long> keep_indices;
    keep_indices.reserve(keep_attributes.size());
    for (const AttributeHandle& h : keep_attributes) {
        keep_indices.emplace_back(h.index);
    }
    std::sort(keep_indices.begin(), keep_indices.end());

    std::vector<bool> mask(m_attributes.size(), false);
    for (const long& i : keep_indices) {
        mask[i] = true;
    }

    std::vector<Attribute<T>> remaining_attributes;
    remaining_attributes.reserve(keep_attributes.size());

    std::vector<long> old_to_new_id(m_attributes.size(), -1);
    for (size_t i = 0, id = 0; i < mask.size(); ++i) {
        if (mask[i]) {
            old_to_new_id[i] = id++;
            remaining_attributes.emplace_back(m_attributes[i]);
            assert(remaining_attributes.size() == id);
        }
    }

    // clean up m_handles
    for (auto it = m_handles.begin(); it != m_handles.end(); /* no increment */) {
        if (!mask[it->second.index]) {
            it = m_handles.erase(it);
        } else {
            it->second.index = old_to_new_id[it->second.index];
            ++it;
        }
    }

    m_attributes = remaining_attributes;
}

template <typename T>
long MeshAttributes<T>::dimension(const AttributeHandle& handle) const
{
    return attribute(handle).dimension();
}


template <typename T>
std::string MeshAttributes<T>::get_name(const AttributeHandle& handle) const
{
    for (const auto& [key, value] : m_handles) {
        if (value == handle) {
            return key;
        }
    }
    throw std::runtime_error("Could not find handle in MeshAttributes");
    return "UNKNOWN";
}

template class MeshAttributes<char>;
template class MeshAttributes<long>;
template class MeshAttributes<double>;
template class MeshAttributes<Rational>;

} // namespace wmtk::attribute
