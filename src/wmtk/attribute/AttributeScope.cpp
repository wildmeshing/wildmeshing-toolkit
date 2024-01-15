#include "AttributeScope.hpp"
#include <wmtk/utils/Rational.hpp>
namespace wmtk::attribute {

template <typename T>
AttributeScope<T>::AttributeScope() = default;
template <typename T>
AttributeScope<T>::~AttributeScope() = default;
template <typename T>
AttributeScope<T>::AttributeScope(std::unique_ptr<AttributeScope>&& parent)
    : m_parent(std::move(parent))
{}


template <typename T>
std::unique_ptr<AttributeScope<T>> AttributeScope<T>::pop_parent()
{
    return std::move(m_parent);
}

template <typename T>
auto AttributeScope<T>::load_const_cached_scalar_value(
    const AccessorBase<T>& accessor,
    int64_t index) const -> T
{
    if (auto it = m_data.find(index); it != m_data.end()) {
        const auto& dat = it->second.data;
        assert(dat.size() == 1);
        return dat(0);
    } else if (m_parent) {
        return m_parent->load_const_cached_scalar_value(accessor, index);
    } else {
        return accessor.const_scalar_attribute(index);
    }
}

template <typename T>
auto AttributeScope<T>::load_cached_scalar_value(AccessorBase<T>& accessor, int64_t index) -> T&
{
    if (auto it = m_data.find(index); it != m_data.end()) {
        auto& dat = it->second.data;
        assert(dat.size() == 1);
        return dat(0);
    } else if (m_parent) {
        return m_parent->load_cached_scalar_value(accessor, index);
    } else {
        return accessor.scalar_attribute(index);
    }
}


template <typename T>
auto AttributeScope<T>::load_cached_vector_value(AccessorBase<T>& accessor, int64_t index)
    -> MapResult
{
    if (auto it = m_data.find(index); it != m_data.end()) {
        auto& dat = it->second.data;
        return MapResult(dat.data(), dat.size());
    } else if (m_parent) {
        return m_parent->load_cached_vector_value(accessor, index);
    } else {
        return accessor.vector_attribute(index);
    }
}
template <typename T>
auto AttributeScope<T>::load_const_cached_vector_value(
    const AccessorBase<T>& accessor,
    int64_t index) const -> ConstMapResult
{
    if (auto it = m_data.find(index); it != m_data.end()) {
        auto& dat = it->second.data;
        return ConstMapResult(dat.data(), dat.size());
    } else if (m_parent) {
        return m_parent->load_const_cached_vector_value(accessor, index);
    } else {
        return accessor.const_vector_attribute(index);
    }
}

template <typename T>
auto AttributeScope<T>::vector_attribute(
    AccessorBase<T>& accessor,
    AttributeAccessMode mode,
    int64_t index) -> MapResult
{
    auto [it, was_inserted] = AttributeCache<T>::load_it(index);
    auto& value = it->second;
    if (was_inserted) {
        if (m_parent) {
            value.data = m_parent->load_const_cached_vector_value(accessor, index);
        } else {
            value.data = accessor.const_vector_attribute(index);
        }
#if !defined(WMTK_ONLY_CACHE_WRITES)
        value.dirty = true;
#endif
        assert(value.data.size() == accessor.dimension());
    }
    return value.data_as_map();
}

template <typename T>
auto AttributeScope<T>::const_vector_attribute(
    const AccessorBase<T>& accessor,
    AttributeAccessMode mode,
    int64_t index) const -> ConstMapResult
{
#if defined(WMTK_ONLY_CACHE_WRITES)
    return load_const_cached_vector_value(accessor, index);
#else

    auto it = AttributeCache<T>::load_it(accessor, mode, index);
    auto& value = it->second;
    if (was_inserted) {
        if (m_parent) {
            value.data = m_parent->load_const_cached_vector_value(accessor, index);
        } else {
            value.data = accessor.const_vector_attribute(index);
        }
    }
    assert(value.data.size() == accessor.dimension());
    return value.data_as_const_map();
#endif
}

template <typename T>
auto AttributeScope<T>::scalar_attribute(
    AccessorBase<T>& accessor,
    AttributeAccessMode mode,
    int64_t index) -> T&
{
    return vector_attribute(accessor, mode, index)(0);
}

template <typename T>
auto AttributeScope<T>::const_scalar_attribute(
    const AccessorBase<T>& accessor,
    AttributeAccessMode mode,
    int64_t index) const -> T
{
    return const_vector_attribute(accessor, mode, index)(0);
}

template <typename T>
void AttributeScope<T>::flush(Attribute<T>& attr)
{
    if (m_parent) {
        AttributeCache<T>::flush_to(*m_parent);
    } else {
        AttributeCache<T>::flush_to(attr);
    }
}
template <typename T>
void AttributeScope<T>::flush_changes_to_vector(const Attribute<T>& attr, std::vector<T>& data)
{
    if (m_parent) {
        m_parent->flush_changes_to_vector(attr, data);
    }
    AttributeCache<T>::flush_to(attr, data);
}

template <typename T>
int64_t AttributeScope<T>::depth() const
{
    if (bool(m_parent)) {
        return 1 + m_parent->depth();
    } else {
        return 1;
    }
}

template class AttributeScope<int64_t>;
template class AttributeScope<double>;
template class AttributeScope<char>;
template class AttributeScope<Rational>;
} // namespace wmtk::attribute
