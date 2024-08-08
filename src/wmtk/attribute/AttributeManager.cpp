#include <spdlog/spdlog.h>

#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/vector_hash.hpp>
#include "AttributeManager.hpp"
#include "PerThreadAttributeScopeStacks.hpp"
namespace wmtk::attribute {
AttributeManager::AttributeManager(int64_t size)
    : m_char_attributes(size)
    , m_long_attributes(size)
    , m_double_attributes(size)
    , m_rational_attributes(size)
    , m_capacities(size, 0)
{}


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

void AttributeManager::serialize(MeshWriter& writer) const
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
void AttributeManager::guarantee_at_least_attributes(int64_t dimension, int64_t size)
{
    assert(dimension < this->size());


    m_char_attributes[dimension].guarantee_at_least(size);
    m_long_attributes[dimension].guarantee_at_least(size);
    m_double_attributes[dimension].guarantee_at_least(size);
    m_rational_attributes[dimension].guarantee_at_least(size);
}

void AttributeManager::guarantee_at_least_attributes(
    const std::vector<int64_t>& at_least_capacities)
{
    assert(at_least_capacities.size() == size());
    for (int64_t dim = 0; dim < size(); ++dim) {
        guarantee_at_least_attributes(dim, at_least_capacities[dim]);
    }
}
void AttributeManager::guarantee_more_attributes(int64_t dimension, int64_t size)
{
    const int64_t current_capacity = m_capacities[dimension];
    const int64_t target_capacity = current_capacity + size;
    guarantee_at_least_attributes(dimension, target_capacity);
}
void AttributeManager::guarantee_more_attributes(const std::vector<int64_t>& more_capacities)
{
    assert(more_capacities.size() == size());
    for (int64_t dim = 0; dim < size(); ++dim) {
        guarantee_more_attributes(dim, more_capacities[dim]);
    }
}
void AttributeManager::set_capacities(std::vector<int64_t> capacities)
{
    assert(capacities.size() == m_capacities.size());
    m_capacities = std::move(capacities);
    reserve_attributes_to_fit();
}
void AttributeManager::assert_capacity_valid() const
{
    assert(m_char_attributes.size() == m_capacities.size());
    assert(m_long_attributes.size() == m_capacities.size());
    assert(m_double_attributes.size() == m_capacities.size());
    assert(m_rational_attributes.size() == m_capacities.size());

    for (size_t i = 0; i < m_capacities.size(); ++i) {
        assert(m_capacities[i] > 0);
        assert(m_char_attributes[i].reserved_size() >= m_capacities[i]);
        assert(m_long_attributes[i].reserved_size() >= m_capacities[i]);
        assert(m_double_attributes[i].reserved_size() >= m_capacities[i]);
        assert(m_rational_attributes[i].reserved_size() >= m_capacities[i]);

        m_char_attributes[i].assert_capacity_valid(m_capacities[i]);
        m_long_attributes[i].assert_capacity_valid(m_capacities[i]);
        m_double_attributes[i].assert_capacity_valid(m_capacities[i]);
        m_rational_attributes[i].assert_capacity_valid(m_capacities[i]);
    }
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
    return m_capacities == other.m_capacities && m_char_attributes == other.m_char_attributes &&
           m_long_attributes == other.m_long_attributes &&
           m_double_attributes == other.m_double_attributes &&
           m_rational_attributes == other.m_rational_attributes;
}

AttributeScopeHandle AttributeManager::create_scope(Mesh& m)
{
    return AttributeScopeHandle(*this);
}

std::string AttributeManager::get_name(
    const attribute::MeshAttributeHandle::HandleVariant& attr) const
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

void AttributeManager::rollback_current_scope()
{
    for (auto& ma : m_char_attributes) {
        ma.rollback_current_scope();
    }
    for (auto& ma : m_long_attributes) {
        ma.rollback_current_scope();
    }
    for (auto& ma : m_double_attributes) {
        ma.rollback_current_scope();
    }
    for (auto& ma : m_rational_attributes) {
        ma.rollback_current_scope();
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

void AttributeManager::change_to_child_scope() const
{
    for (auto& ma : m_char_attributes) {
        ma.change_to_child_scope();
    }
    for (auto& ma : m_long_attributes) {
        ma.change_to_child_scope();
    }
    for (auto& ma : m_double_attributes) {
        ma.change_to_child_scope();
    }
    for (auto& ma : m_rational_attributes) {
        ma.change_to_child_scope();
    }
}

std::vector<MeshAttributeHandle::HandleVariant> AttributeManager::get_all_attributes() const
{
    std::vector<MeshAttributeHandle::HandleVariant> handles;

    auto run = [&](auto type) {
        using T = std::decay_t<decltype(type)>;

        const std::vector<MeshAttributes<T>>& mesh_attributes = get<T>();
        for (size_t pt_index = 0; pt_index < mesh_attributes.size(); ++pt_index) {
            size_t count = mesh_attributes[pt_index].attribute_count();
            for (int64_t index = 0; index < count; ++index) {
                TypedAttributeHandle<T> t;
                t.m_base_handle.index = index;
                t.m_primitive_type = get_primitive_type_from_id(pt_index);
                handles.emplace_back(t);
            }
        }
    };
    run(double{});
    run(int64_t{});
    run(char{});
    run(Rational{});
    return handles;
}

namespace {
template <typename T>
class ClearAttrDataT : public std::array<std::vector<AttributeHandle>, 5>
{
};

class ClearAttrData : public ClearAttrDataT<char>,
                      public ClearAttrDataT<int64_t>,
                      public ClearAttrDataT<double>,
                      public ClearAttrDataT<Rational>
{
public:
    template <typename T>
    ClearAttrDataT<T>& get();
};
template <typename T>
ClearAttrDataT<T>& ClearAttrData::get()
{
    return static_cast<ClearAttrDataT<T>&>(*this);
}
} // namespace
void AttributeManager::clear_attributes(
    const std::vector<attribute::MeshAttributeHandle::HandleVariant>& custom_attributes)
{
    // std::array<std::array<std::vector<AttributeHandle>, 5>, 4>
    //    keeps; // [char/int64_t/...][ptype][attribute]


    ClearAttrData customs;
    for (const attribute::MeshAttributeHandle::HandleVariant& attr : custom_attributes) {
        std::visit(
            [&](auto&& val) noexcept {
                using HandleType = typename std::decay_t<decltype(val)>;
                if constexpr (attribute::MeshAttributeHandle::template handle_type_is_basic<
                                  HandleType>()) {
                    using T = typename HandleType::Type;
                    customs.get<T>()[get_primitive_type_id(val.primitive_type())].emplace_back(
                        val.base_handle());
                }
            },
            attr);
    }


    auto run = [&](auto t) {
        using T = typename std::decay_t<decltype(t)>;
        auto& mycustoms = customs.get<T>();

        for (size_t ptype_id = 0; ptype_id < m_char_attributes.size(); ++ptype_id) {
            const PrimitiveType primitive_type = get_primitive_type_from_id(ptype_id);


            get<T>(primitive_type).remove_attributes(mycustoms[ptype_id]);
        }
    };

    run(double{});
    run(int64_t{});
    run(char{});
    run(Rational{});
}
void AttributeManager::delete_attribute(
    const attribute::MeshAttributeHandle::HandleVariant& to_delete)
{
    std::visit(
        [&](auto&& val) noexcept {
            using HandleType = typename std::decay_t<decltype(val)>;
            if constexpr (attribute::MeshAttributeHandle::template handle_type_is_basic<
                              HandleType>()) {
                using T = typename HandleType::Type;
                get<T>(val).remove_attribute(val.base_handle());
            }
        },
        to_delete);
}

} // namespace wmtk::attribute
