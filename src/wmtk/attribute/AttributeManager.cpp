#include "AttributeManager.hpp"
#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include "PerThreadAttributeScopeStacks.hpp"
namespace wmtk::attribute {
AttributeManager::AttributeManager(long size)
    : m_char_attributes(size)
    , m_long_attributes(size)
    , m_double_attributes(size)
    , m_capacities(size, 0)
{}

AttributeManager::AttributeManager(const AttributeManager& o) = default;
AttributeManager::AttributeManager(AttributeManager&& o) = default;
AttributeManager& AttributeManager::operator=(const AttributeManager& o) = default;
AttributeManager& AttributeManager::operator=(AttributeManager&& o) = default;


AttributeManager::~AttributeManager() = default;

void AttributeManager::serialize(MeshWriter& writer)
{
    for (long dim = 0; dim < m_capacities.size(); ++dim) {
        if (!writer.write(dim)) continue;
        m_char_attributes[dim].serialize(dim, writer);
        m_long_attributes[dim].serialize(dim, writer);
        m_double_attributes[dim].serialize(dim, writer);
    }
    // now that the WMTK link exists we can write hte capacities to that link
    writer.write_capacities(m_capacities);
}

void AttributeManager::reserve_to_fit()
{
    for (long dim = 0; dim < m_capacities.size(); ++dim) {
        const long capacity = m_capacities[dim];
        reserve_attributes(dim, capacity);
    }
}
void AttributeManager::reserve_attributes(long dimension, long capacity)
{
    m_char_attributes[dimension].reserve(capacity);
    m_long_attributes[dimension].reserve(capacity);
    m_double_attributes[dimension].reserve(capacity);
}
void AttributeManager::set_capacities(std::vector<long> capacities)
{
    assert(capacities.size() == m_capacities.size());
    m_capacities = std::move(capacities);
    reserve_attributes_to_fit();
}

void AttributeManager::reserve_attributes_to_fit()
{
    for (long j = 0; j < size(); ++j) {
        reserve_attributes(j, m_capacities[j]);
    }
}


long AttributeManager::size() const
{
    return long(m_capacities.size());
}
bool AttributeManager::operator==(const AttributeManager& other) const
{
    return m_capacities == other.m_capacities && m_char_attributes == other.m_char_attributes &&
           m_long_attributes == other.m_long_attributes &&
           m_double_attributes == other.m_double_attributes;
}

AttributeScopeHandle AttributeManager::create_scope(Mesh& m)
{
    return AttributeScopeHandle(*this);
}

void AttributeManager::push_scope()
{
    for (auto& ma : m_char_attributes) {
        ma.push_scope();
    }
    for (auto& ma : m_long_attributes) {
        ma.push_scope();
    }
    for (auto& ma : m_double_attributes) {
        ma.push_scope();
    }
    for (auto& ma : m_rational_attributes) {
        ma.push_scope();
    }
}
void AttributeManager::pop_scope(bool apply_updates)
{
    for (auto& ma : m_char_attributes) {
        ma.pop_scope(apply_updates);
    }
    for (auto& ma : m_long_attributes) {
        ma.pop_scope(apply_updates);
    }
    for (auto& ma : m_double_attributes) {
        ma.pop_scope(apply_updates);
    }
    for (auto& ma : m_rational_attributes) {
        ma.pop_scope(apply_updates);
    }
}

void AttributeManager::clear_current_scope()
{
    for (auto& ma : m_char_attributes) {
        ma.clear_current_scope();
    }
    for (auto& ma : m_long_attributes) {
        ma.clear_current_scope();
    }
    for (auto& ma : m_double_attributes) {
        ma.clear_current_scope();
    }
    for (auto& ma : m_rational_attributes) {
        ma.clear_current_scope();
    }
}
} // namespace wmtk::attribute
