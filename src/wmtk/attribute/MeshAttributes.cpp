#include "MeshAttributes.hpp"
#include <wmtk/attribute/internal/hash.hpp>
#include <wmtk/utils/Hashable.hpp>
#include "PerThreadAttributeScopeStacks.hpp"

#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

#include <cassert>
#include <functional>
#include <stdexcept>
#include <string>
#include <utility>

namespace wmtk::attribute {


template <typename T>
void MeshAttributes<T>::serialize(const int dim, MeshWriter& writer) const
{
    for (const auto& p : m_handles) {
        const auto& handle = p.second;
        const auto& attr = *m_attributes[handle.index];
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
    for (auto& attr_ptr : m_attributes) {
        if (bool(attr_ptr)) {
            attr_ptr->push_scope();
        }
    }
}
template <typename T>
void MeshAttributes<T>::pop_scope(bool apply_updates)
{
    for (auto& attr_ptr : m_attributes) {
        if (bool(attr_ptr)) {
            attr_ptr->pop_scope(apply_updates);
        }
    }
}
template <typename T>
void MeshAttributes<T>::rollback_current_scope()
{
    for (auto& attr_ptr : m_attributes) {
        if (bool(attr_ptr)) {
            attr_ptr->rollback_current_scope();
        }
    }
}
template <typename T>
void MeshAttributes<T>::change_to_parent_scope() const
{
    for (const auto& attr_ptr : m_attributes) {
        if (bool(attr_ptr)) {
            auto& stack = attr_ptr->get_local_scope_stack();

            stack.change_to_next_scope();
        }
    }
}

template <typename T>
void MeshAttributes<T>::change_to_child_scope() const
{
    for (const auto& attr_ptr : m_attributes) {
        if (bool(attr_ptr)) {
            attr_ptr->get_local_scope_stack().change_to_previous_scope();
        }
    }
}

template <typename T>
AttributeHandle MeshAttributes<T>::register_attribute(
    const std::string& name,
    int64_t dimension,
    bool replace,
    T default_value)
{
    if (!replace && m_handles.find(name) != m_handles.end()) {
        log_and_throw_error(
            "Cannot register attribute '{}' because it exists already. Set replace to true if you "
            "want to overwrite the attribute",
            name);
    }

    AttributeHandle handle;


    if (replace && m_handles.find(name) != m_handles.end()) {
        auto it = m_handles.find(name);
        handle.index = it->second.index;
    } else {
        handle.index = m_attributes.size();
        m_attributes.emplace_back(
            std::make_unique<Attribute<T>>(name, dimension, default_value, reserved_size()));
    }
    m_handles[name] = handle;


    return handle;
}

template <typename T>
void MeshAttributes<T>::assert_capacity_valid(int64_t cap) const
{
    for (const auto& a : m_attributes) {
        if (bool(a)) {
            assert(a->reserved_size() >= cap);
        }
    }
}
template <typename T>
AttributeHandle MeshAttributes<T>::attribute_handle(const std::string& name) const
{
    return m_handles.at(name);
}
template <typename T>
bool MeshAttributes<T>::has_attribute(const std::string& name) const
{
    auto it = m_handles.find(name);
    return it != m_handles.end() && bool(m_attributes[it->second.index]);
}

template <typename T>
bool MeshAttributes<T>::operator==(const MeshAttributes<T>& other) const
{
    if (m_handles != other.m_handles) {
        return false;
    }
    if (m_attributes.size() != other.m_attributes.size()) {
        return false;
    }
    for (size_t j = 0; j < m_attributes.size(); ++j) {
        const bool exists = bool(m_attributes[j]);
        const bool o_exists = bool(other.m_attributes[j]);
        if (exists != o_exists) {
            return false;
        } else if (exists && o_exists && !(*m_attributes[j] == *other.m_attributes[j])) {
            return false;
        }
    }
    return true;
}


template <typename T>
void MeshAttributes<T>::set(const AttributeHandle& handle, std::vector<T> val)
{
    // TODO: should we validate the size of val compared to the internally held data?
    auto& attr_ptr = m_attributes[handle.index];
    assert(bool(attr_ptr));
    auto& attr = *attr_ptr;
    attr.set(std::move(val));
}

template <typename T>
size_t MeshAttributes<T>::attribute_size(const AttributeHandle& handle) const
{
    auto& attr_ptr = m_attributes[handle.index];
    assert(bool(attr_ptr));
    return attr_ptr->reserved_size();
}

template <typename T>
int64_t MeshAttributes<T>::reserved_size() const
{
    return m_reserved_size;
}

template <typename T>
size_t MeshAttributes<T>::attribute_count() const
{
    return active_attributes().size();
}
template <typename T>
auto MeshAttributes<T>::active_attributes() const -> std::vector<AttributeHandle>
{
    std::vector<AttributeHandle> handles;
    handles.reserve(m_attributes.size());
    for(size_t j = 0; j < m_attributes.size(); ++j) {
        if(bool(m_attributes[j])) {
            handles.emplace_back(j);
        }
    }

    return handles;

}

template <typename T>
void MeshAttributes<T>::reserve(const int64_t size)
{
    m_reserved_size = size;
    for (auto& attr_ptr : m_attributes) {
        if (bool(attr_ptr)) {
            attr_ptr->reserve(size);
        }
    }
}

template <typename T>
void MeshAttributes<T>::reserve_more(const int64_t size)
{
    reserve(m_reserved_size + size);
}

template <typename T>
void MeshAttributes<T>::guarantee_at_least(const int64_t size)
{
    if (size > m_reserved_size) {
        // logger().warn("Pre-reserve enough simplices before your operation.");
        reserve(size);
    }
}

template <typename T>
void MeshAttributes<T>::remove_attributes(const std::vector<AttributeHandle>& attributes, bool invalidate_handles)
{
    if (attributes.empty()) {
        return;
    }
    std::vector<int64_t> remove_indices;
    std::transform(
        attributes.begin(),
        attributes.end(),
        std::back_inserter(remove_indices),
        [](const AttributeHandle& h) { return h.index; });
    std::sort(remove_indices.begin(), remove_indices.end());
    remove_indices.erase(
        std::unique(remove_indices.begin(), remove_indices.end()),
        remove_indices.end());


    for (const int64_t& i : remove_indices) {
        m_attributes[i].reset();
    }


    if(invalidate_handles) {
    clear_dead_attributes();
    }
}


template <typename T>
void MeshAttributes<T>::remove_attribute(const AttributeHandle& attribute)
{
    m_attributes[attribute.index].reset();
}

template <typename T>
void MeshAttributes<T>::clear_dead_attributes()
{
    size_t old_index = 0;
    size_t new_index = 0;
    std::vector<int64_t> old_to_new_id(m_attributes.size(), -1);
    for (old_index = 0; old_index < m_attributes.size(); ++old_index) {
        if (bool(m_attributes[old_index])) {
            old_to_new_id[old_index] = new_index;
            m_attributes[new_index++] = std::move(m_attributes[old_index]);
        }
    }
    m_attributes.resize(new_index);

    // clean up m_handles
    for (auto it = m_handles.begin(); it != m_handles.end(); /* no increment */) {
        if (int64_t new_ind = old_to_new_id[it->second.index]; new_ind == -1) {
            it = m_handles.erase(it);
        } else {
            it->second.index = new_ind;
            ++it;
        }
    }
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

template <typename T>
void MeshAttributes<T>::set_name(const AttributeHandle& handle, const std::string& name)
{
    const std::string old_name = get_name(handle);

    if (old_name == name) {
        return;
    }

    auto& attr = m_attributes[handle.index];
    assert(bool(attr));
    assert(attr->m_name == old_name);

    assert(m_handles.count(name) == 0); // name should not exist already

    attr->m_name = name;
    m_handles[name] = m_handles[old_name];
    m_handles.erase(old_name);
}

template class MeshAttributes<char>;
template class MeshAttributes<int64_t>;
template class MeshAttributes<double>;
template class MeshAttributes<Rational>;

} // namespace wmtk::attribute
