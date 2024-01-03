#include "AttributeManager.hpp"
#include <fmt/format.h>
#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/vector_hash.hpp>
#include "PerThreadAttributeScopeStacks.hpp"
namespace wmtk::attribute {
AttributeManager::AttributeManager(int64_t size)
    : m_char_attributes(size)
    , m_long_attributes(size)
    , m_double_attributes(size)
    , m_rational_attributes(size)
    , m_capacities(size, 0)
{}

AttributeManager::AttributeManager(const AttributeManager& o) = default;
AttributeManager::AttributeManager(AttributeManager&& o) = default;
AttributeManager& AttributeManager::operator=(const AttributeManager& o) = default;
AttributeManager& AttributeManager::operator=(AttributeManager&& o) = default;

// attribute directly hashes its "child_hashables" components so it overrides "child_hashes"
std::map<std::string, const wmtk::utils::Hashable*> AttributeManager::child_hashables() const
{
    std::map<std::string, const wmtk::utils::Hashable*> ret;
    for (size_t j = 0; j < m_char_attributes.size(); ++j) {
        ret[fmt::format("char_attributes_{}", j)] = &m_char_attributes[j];
    }
    for (size_t j = 0; j < m_char_attributes.size(); ++j) {
        ret[fmt::format("char_attributes_{}", j)] = &m_char_attributes[j];
    }
    for (size_t j = 0; j < m_long_attributes.size(); ++j) {
        ret[fmt::format("long_attributes_{}", j)] = &m_long_attributes[j];
    }
    for (size_t j = 0; j < m_double_attributes.size(); ++j) {
        ret[fmt::format("double_attributes_{}", j)] = &m_double_attributes[j];
    }
    for (size_t j = 0; j < m_rational_attributes.size(); ++j) {
        ret[fmt::format("rational_attributes_{}", j)] = &m_rational_attributes[j];
    }
    return ret;
}
std::map<std::string, std::size_t> AttributeManager::child_hashes() const
{
    // default implementation pulls the child attributes (ie the attributes)
    std::map<std::string, std::size_t> ret = wmtk::utils::MerkleTreeInteriorNode::child_hashes();

    // hash handle data
    for (size_t j = 0; j < m_capacities.size(); ++j) {
        ret[fmt::format("capacities_{}", j)] = m_capacities[j];
    }
    return ret;
}


AttributeManager::~AttributeManager() = default;

void AttributeManager::serialize(MeshWriter& writer)
{
    for (int64_t dim = 0; dim < m_capacities.size(); ++dim) {
        if (!writer.write(dim)) continue;
        m_char_attributes[dim].serialize(dim, writer);
        m_long_attributes[dim].serialize(dim, writer);
        m_double_attributes[dim].serialize(dim, writer);
        m_rational_attributes[dim].serialize(dim, writer);
    }
    // now that the WMTK link exists we can write hte capacities to that link
    writer.write_capacities(m_capacities);
}

void AttributeManager::reserve_to_fit()
{
    for (int64_t dim = 0; dim < m_capacities.size(); ++dim) {
        const int64_t capacity = m_capacities[dim];
        reserve_attributes(dim, capacity);
    }
}
void AttributeManager::reserve_attributes(int64_t dimension, int64_t capacity)
{
    m_char_attributes[dimension].reserve(capacity);
    m_long_attributes[dimension].reserve(capacity);
    m_double_attributes[dimension].reserve(capacity);
    m_rational_attributes[dimension].reserve(capacity);
}

void AttributeManager::reserve_more_attributes(int64_t dimension, int64_t size)
{
    assert(dimension < this->size());
    m_char_attributes[dimension].reserve_more(size);
    m_long_attributes[dimension].reserve_more(size);
    m_double_attributes[dimension].reserve_more(size);
    m_rational_attributes[dimension].reserve_more(size);
}
void AttributeManager::reserve_more_attributes(const std::vector<int64_t>& more_capacities)
{
    assert(more_capacities.size() == size());
    for (int64_t dim = 0; dim < size(); ++dim) {
        reserve_more_attributes(dim, more_capacities[dim]);
    }
}
void AttributeManager::set_capacities(std::vector<int64_t> capacities)
{
    assert(capacities.size() == m_capacities.size());
    m_capacities = std::move(capacities);
    reserve_attributes_to_fit();
}

void AttributeManager::reserve_attributes_to_fit()
{
    for (int64_t j = 0; j < size(); ++j) {
        reserve_attributes(j, m_capacities[j]);
    }
}


int64_t AttributeManager::size() const
{
    return int64_t(m_capacities.size());
}
bool AttributeManager::operator==(const AttributeManager& other) const
{
    bool same_cap = m_capacities == other.m_capacities;
    bool same_char = m_char_attributes == other.m_char_attributes;
    bool same_long = m_long_attributes == other.m_long_attributes;
    bool same_double = m_double_attributes == other.m_double_attributes;
    bool same_rational = m_rational_attributes == other.m_rational_attributes;

    return m_capacities == other.m_capacities && m_char_attributes == other.m_char_attributes &&
           m_long_attributes == other.m_long_attributes &&
           m_double_attributes == other.m_double_attributes &&
           m_rational_attributes == other.m_rational_attributes;
}

AttributeScopeHandle AttributeManager::create_scope(Mesh& m)
{
    return AttributeScopeHandle(*this);
}

std::string AttributeManager::get_name(const attribute::TypedAttributeHandleVariant& attr) const
{
    std::string name = std::visit(
        [&](auto&& val) {
            using T = std::decay_t<decltype(val)>;
            return this->get_name(std::get<T>(attr));
        },
        attr);

    return name;
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

void AttributeManager::change_to_parent_scope() const
{
    for (auto& ma : m_char_attributes) {
        ma.change_to_parent_scope();
    }
    for (auto& ma : m_long_attributes) {
        ma.change_to_parent_scope();
    }
    for (auto& ma : m_double_attributes) {
        ma.change_to_parent_scope();
    }
    for (auto& ma : m_rational_attributes) {
        ma.change_to_parent_scope();
    }
}

void AttributeManager::change_to_leaf_scope() const
{
    for (auto& ma : m_char_attributes) {
        ma.change_to_leaf_scope();
    }
    for (auto& ma : m_long_attributes) {
        ma.change_to_leaf_scope();
    }
    for (auto& ma : m_double_attributes) {
        ma.change_to_leaf_scope();
    }
    for (auto& ma : m_rational_attributes) {
        ma.change_to_leaf_scope();
    }
}

void AttributeManager::clear_attributes(
    std::vector<attribute::TypedAttributeHandleVariant> keep_attributes)
{
    std::array<std::array<std::vector<AttributeHandle>, 5>, 4>
        keeps; // [char/int64_t/...][ptype][attribute]

    for (const attribute::TypedAttributeHandleVariant& attr : keep_attributes) {
        std::visit(
            [&](auto&& val) {
                using T = typename std::decay_t<decltype(val)>::Type;

                int64_t type_id = -1;

                if constexpr (std::is_same_v<T, char>) {
                    type_id = 0;
                }
                if constexpr (std::is_same_v<T, int64_t>) {
                    type_id = 1;
                }
                if constexpr (std::is_same_v<T, double>) {
                    type_id = 2;
                }
                if constexpr (std::is_same_v<T, Rational>) {
                    type_id = 3;
                }

                assert(type_id != -1);

                keeps[type_id][get_primitive_type_id(val.primitive_type())].emplace_back(
                    val.base_handle());
            },
            attr);
    }

    std::array<std::array<std::vector<AttributeHandle>, 5>, 4>
        customs; // [char/int64_t/...][ptype][attribute]

    for (const attribute::TypedAttributeHandleVariant& attr : m_custom_attributes) {
        std::visit(
            [&](auto&& val) {
                using T = typename std::decay_t<decltype(val)>::Type;

                int64_t type_id = -1;

                if constexpr (std::is_same_v<T, char>) {
                    type_id = 0;
                }
                if constexpr (std::is_same_v<T, int64_t>) {
                    type_id = 1;
                }
                if constexpr (std::is_same_v<T, double>) {
                    type_id = 2;
                }
                if constexpr (std::is_same_v<T, Rational>) {
                    type_id = 3;
                }

                assert(type_id != -1);

                customs[type_id][get_primitive_type_id(val.primitive_type())].emplace_back(
                    val.base_handle());
            },
            attr);
    }

    for (size_t type_id = 0; type_id < keeps.size(); ++type_id) {
        for (size_t ptype_id = 0; ptype_id < m_char_attributes.size(); ++ptype_id) {
            std::vector<AttributeHandle> diff_char;
            std::vector<AttributeHandle> keeps_char = keeps[type_id][ptype_id];
            std::vector<AttributeHandle> customs_char = customs[type_id][ptype_id];

            std::sort(keeps_char.begin(), keeps_char.end());

            std::sort(customs_char.begin(), customs_char.end());

            std::set_difference(
                customs_char.begin(),
                customs_char.end(),
                keeps_char.begin(),
                keeps_char.end(),
                std::inserter(diff_char, diff_char.begin()));

            switch (type_id) {
            case 0:
                get<char>(get_primitive_type_from_id(ptype_id)).remove_attributes(diff_char);
                break;
            case 1:
                get<int64_t>(get_primitive_type_from_id(ptype_id)).remove_attributes(diff_char);
                break;
            case 2:
                get<double>(get_primitive_type_from_id(ptype_id)).remove_attributes(diff_char);
                break;
            case 3:
                get<Rational>(get_primitive_type_from_id(ptype_id)).remove_attributes(diff_char);
                break;
            default: throw std::runtime_error("unknown type"); break;
            }
        }
    }

    // clean up m_custom_attributes
    m_custom_attributes = keep_attributes;
}

} // namespace wmtk::attribute
