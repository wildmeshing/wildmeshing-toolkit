#include "TypedAttributeManager.hpp"
#include <wmtk/attribute/internal/hash.hpp>
#include <wmtk/utils/Hashable.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/io/HDF5Writer.hpp>
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
void TypedAttributeManager<T>::serialize(const int dim, MeshWriter& writer) const
{
    std::vector<std::string> attribute_names;
    std::transform(
        m_attributes.begin(),
        m_attributes.end(),
        std::back_inserter(attribute_names),
        [](const auto& ptr) -> std::string {
            if (ptr) {
                return ptr->name();
            } else {
                return {};
            }
        });

    attribute_names.erase(
        std::remove_if(
            attribute_names.begin(),
            attribute_names.end(),
            [](const std::string& name) { return name.empty(); }),
        attribute_names.end());

    HDF5Writer* hdf5_w = dynamic_cast<HDF5Writer*>(&writer);
    if (hdf5_w != nullptr) {
        hdf5_w->write_attribute_names<T>(dim, attribute_names);
    }

    for (const auto& attr_ptr : m_attributes) {
        if (bool(attr_ptr)) {
            attr_ptr->serialize(dim, writer);
        }
    }
}

template <typename T>
std::map<std::string, std::size_t> TypedAttributeManager<T>::child_hashes() const
{
    // default implementation pulls the child attributes (ie the attributes)
    std::map<std::string, std::size_t> ret = wmtk::utils::MerkleTreeInteriorNode::child_hashes();

    for (int64_t i = 0; i < m_attributes.size(); ++i) {
        if (!m_attributes[i]) {
            continue;
        }
        const std::string& name = m_attributes[i]->name();
        AttributeHandle handle(i);
        ret["attr_handle_" + name] = std::hash<AttributeHandle>{}(handle);
    }

    return ret;
}
template <typename T>
std::map<std::string, const wmtk::utils::Hashable*> TypedAttributeManager<T>::child_hashables()
    const

{
    std::map<std::string, const wmtk::utils::Hashable*> ret;

    for (int64_t i = 0; i < m_attributes.size(); ++i) {
        if (!m_attributes[i]) {
            continue;
        }
        const std::string& name = m_attributes[i]->name();
        AttributeHandle handle(i);
        const auto& attr = attribute(handle);

        ret["attr_" + name] = &attr;
    }

    return ret;
}

template <typename T>
void TypedAttributeManager<T>::push_scope()
{
    for (auto& attr_ptr : m_attributes) {
        if (bool(attr_ptr)) {
            attr_ptr->push_scope();
        }
    }
}
template <typename T>
void TypedAttributeManager<T>::pop_scope(bool apply_updates)
{
    for (auto& attr_ptr : m_attributes) {
        if (bool(attr_ptr)) {
            attr_ptr->pop_scope(apply_updates);
        }
    }
}
template <typename T>
void TypedAttributeManager<T>::rollback_current_scope()
{
    for (auto& attr_ptr : m_attributes) {
        if (bool(attr_ptr)) {
            attr_ptr->rollback_current_scope();
        }
    }
}
template <typename T>
void TypedAttributeManager<T>::change_to_parent_scope() const
{
    for (const auto& attr_ptr : m_attributes) {
        if (bool(attr_ptr)) {
            attr_ptr->change_to_next_scope();
        }
    }
}

template <typename T>
void TypedAttributeManager<T>::change_to_child_scope() const
{
    for (const auto& attr_ptr : m_attributes) {
        if (bool(attr_ptr)) {
            attr_ptr->change_to_previous_scope();
        }
    }
}

template <typename T>
AttributeHandle TypedAttributeManager<T>::register_attribute(
    const std::string& name,
    int64_t dimension,
    bool replace,
    T default_value)
{
    if (!replace && has_attribute(name)) {
        log_and_throw_error(
            "Cannot register attribute '{}' because it exists already. Set replace to true if you "
            "want to overwrite the attribute",
            name);
    }

    if (replace) {
        for (int64_t i = 0; i < m_attributes.size(); ++i) {
            if (!m_attributes[i]) {
                continue;
            }
            if (m_attributes[i]->name() == name) {
                return i;
            }
        }
    }

    AttributeHandle handle(m_attributes.size());
    m_attributes.emplace_back(
        std::make_unique<CachingAttribute<T>>(name, dimension, default_value, reserved_size()));

    return handle;
}

template <typename T>
void TypedAttributeManager<T>::assert_capacity_valid(int64_t cap) const
{
    for (const auto& a : m_attributes) {
        if (bool(a)) {
            assert(a->reserved_size() >= cap);
        }
    }
}
template <typename T>
AttributeHandle TypedAttributeManager<T>::attribute_handle(const std::string& name) const
{
    for (int64_t i = 0; i < m_attributes.size(); ++i) {
        if (!m_attributes[i]) {
            continue;
        }
        if (m_attributes[i]->name() == name) {
            return i;
        }
    }

    log_and_throw_error("Cannot find handle for attribute named {}", name);
}
template <typename T>
bool TypedAttributeManager<T>::has_attribute(const std::string& name) const
{
    for (int64_t i = 0; i < m_attributes.size(); ++i) {
        if (!m_attributes[i]) {
            continue;
        }
        if (m_attributes[i]->name() == name) {
            return true;
        }
    }

    return false;
}

template <typename T>
bool TypedAttributeManager<T>::operator==(const TypedAttributeManager<T>& other) const
{
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
void TypedAttributeManager<T>::set(const AttributeHandle& handle, std::vector<T> val)
{
    // TODO: should we validate the size of val compared to the internally held data?
    auto& attr_ptr = m_attributes[handle.index()];
    assert(bool(attr_ptr));
    auto& attr = *attr_ptr;
    attr.set(std::move(val));
}

template <typename T>
size_t TypedAttributeManager<T>::attribute_size(const AttributeHandle& handle) const
{
    auto& attr_ptr = m_attributes[handle.index()];
    assert(bool(attr_ptr));
    return attr_ptr->reserved_size();
}

template <typename T>
int64_t TypedAttributeManager<T>::reserved_size() const
{
    return m_reserved_size;
}

template <typename T>
size_t TypedAttributeManager<T>::attribute_count() const
{
    return active_attributes().size();
}
template <typename T>
auto TypedAttributeManager<T>::active_attributes() const -> std::vector<AttributeHandle>
{
    std::vector<AttributeHandle> handles;
    handles.reserve(m_attributes.size());
    for (size_t j = 0; j < m_attributes.size(); ++j) {
        if (bool(m_attributes[j])) {
            handles.emplace_back(AttributeHandle(j));
        }
    }

    return handles;
}
template <typename T>
bool TypedAttributeManager<T>::is_active(const AttributeHandle& h) const
{
    const size_t index = h.index();
    assert(index < m_attributes.size());
    return bool(m_attributes[index]);
}

template <typename T>
void TypedAttributeManager<T>::reserve(const int64_t size)
{
    m_reserved_size = size;
    for (auto& attr_ptr : m_attributes) {
        if (bool(attr_ptr)) {
            attr_ptr->reserve(size);
        }
    }
}

template <typename T>
void TypedAttributeManager<T>::reserve_more(const int64_t size)
{
    reserve(m_reserved_size + size);
}

template <typename T>
void TypedAttributeManager<T>::guarantee_at_least(const int64_t size)
{
    if (size > m_reserved_size) {
        // logger().warn("Pre-reserve enough simplices before your operation.");
        reserve(size);
    }
}

template <typename T>
void TypedAttributeManager<T>::remove_attributes(const std::vector<AttributeHandle>& attributes)
{
    for (const AttributeHandle& i : attributes) {
        remove_attribute(i);
    }
}


template <typename T>
void TypedAttributeManager<T>::remove_attribute(const AttributeHandle& attribute)
{
    auto& attr = m_attributes[attribute.index()];
    if (!attr) {
        logger().warn("Attribute was already deleted.");
        return;
    }

    m_attributes[attribute.index()].reset();
}

template <typename T>
bool TypedAttributeManager<T>::validate_handle(const AttributeHandle& handle) const
{
    if (handle.index() >= m_attributes.size()) {
        logger().warn("Handle index is larger than the attribute vector");
        return false;
    }

    if (!m_attributes[handle.index()]) {
        return false;
    }

    return true;
}

template <typename T>
void TypedAttributeManager<T>::clear_dead_attributes()
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
}
template <typename T>
std::string TypedAttributeManager<T>::get_name(const AttributeHandle& handle) const
{
    assert(handle.index() < m_attributes.size());
    if (m_attributes[handle.index()]) {
        return m_attributes[handle.index()]->name();
    }
    throw std::runtime_error("Could not find handle in TypedAttributeManager");
    return "UNKNOWN";
}

template <typename T>
void TypedAttributeManager<T>::set_name(const AttributeHandle& handle, const std::string& name)
{
    const std::string old_name = get_name(handle);

    if (old_name == name) {
        return;
    }

    auto& attr = m_attributes[handle.index()];
    assert(bool(attr));
    assert(attr->name() == old_name);

    assert(!has_attribute(name)); // name should not exist already

    attr->set_name(name);
}

template class TypedAttributeManager<char>;
template class TypedAttributeManager<int64_t>;
template class TypedAttributeManager<double>;
template class TypedAttributeManager<Rational>;

} // namespace wmtk::attribute
