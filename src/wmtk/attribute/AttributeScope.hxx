#include "AttributeScope.hpp"
#include <wmtk/utils/Rational.hpp>
namespace wmtk::attribute {

template <typename T>
AttributeScope<T>::AttributeScope() {
}
template <typename T>
AttributeScope<T>::~AttributeScope() {

}
template <typename T>
AttributeScope<T>::AttributeScope(std::unique_ptr<AttributeScope>&& next)
    : m_next(std::move(next))
{
    if (bool(m_next)) {
        m_next->m_previous = this;
    }
}


template <typename T>
std::unique_ptr<AttributeScope<T>> AttributeScope<T>::pop_to_next()
{
    if (m_next) {
        m_next->m_previous = nullptr;
        AttributeCache<T>::flush_to(*m_next);
    }
    return std::move(m_next);
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
#if defined(WMTK_FLUSH_ON_FAIL)
    } else if (m_previous) {
        return m_previous->load_const_cached_scalar_value(accessor, index);
#else
    } else if (m_next) {
        return m_next->load_const_cached_scalar_value(accessor, index);
#endif
    } else {
        return accessor.const_scalar_attribute(index);
    }
}

#if !defined(WMTK_FLUSH_ON_FAIL)
template <typename T>
auto AttributeScope<T>::load_cached_scalar_value(AccessorBase<T>& accessor, int64_t index) -> T&
{
    if (auto it = m_data.find(index); it != m_data.end()) {
        auto& dat = it->second.data;
        assert(dat.size() == 1);
        return dat(0);
#if defined(WMTK_FLUSH_ON_FAIL)
    } else if (m_previous) {
        return m_previous->load_cached_scalar_value(accessor, index);
#else
    } else if (m_next) {
        return m_next->load_cached_scalar_value(accessor, index);
#endif
    } else {
        return accessor.scalar_attribute(index);
    }
}
#endif


#if !defined(WMTK_FLUSH_ON_FAIL)
template <typename T>
auto AttributeScope<T>::load_cached_vector_value(AccessorBase<T>& accessor, int64_t index)
    -> MapResult
{
    if (auto it = m_data.find(index); it != m_data.end()) {
        auto& dat = it->second.data;
        return MapResult(dat.data(), dat.size());
    } else if (m_next) {
        return m_next->load_cached_vector_value(accessor, index);
    } else {
        return accessor.vector_attribute(index);
    }
}
#endif
template <typename T>
auto AttributeScope<T>::load_const_cached_vector_value(
    const AccessorBase<T>& accessor,
    int64_t index) const -> ConstMapResult
{
    if (auto it = m_data.find(index); it != m_data.end()) {
        auto& dat = it->second.data;
        auto v = ConstMapResult(dat.data(), dat.size());
        return v;
#if defined(WMTK_FLUSH_ON_FAIL)
    } else if (m_previous) {
        auto v = m_previous->load_const_cached_vector_value(accessor, index);
        return v;
#else
    } else if (m_next) {
        return m_next->load_const_cached_vector_value(accessor, index);
#endif
    } else {

        auto v = accessor.const_vector_attribute(index);
        return v;
    }
}

#if !defined(WMTK_FLUSH_ON_FAIL)
template <typename T>
auto AttributeScope<T>::vector_attribute(AccessorBase<T>& accessor, int64_t index) -> MapResult
{
    auto [it, was_inserted] = AttributeCache<T>::load_it(index);
    auto& value = it->second;
    if (was_inserted) {
#if defined(WMTK_FLUSH_ON_FAIL)
        if (m_previous) {
            value.data = m_previous->load_const_cached_vector_value(accessor, index);
#else
        if (m_next) {
            value.data = m_next->load_const_cached_vector_value(accessor, index);
#endif
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
#endif

template <typename T>
auto AttributeScope<T>::const_vector_attribute(const AccessorBase<T>& accessor, int64_t index) const
    -> ConstMapResult
{
#if defined(WMTK_ONLY_CACHE_WRITES)
    return load_const_cached_vector_value(accessor, index);
#else

#if defined(WMTK_FLUSH_ON_FAIL)

    return load_const_cached_vector_value(accessor,index);
#else
    auto it = AttributeCache<T>::load_it(accessor, index);
    auto& value = it->second;
    if (was_inserted) {
        if (m_next) {
            value.data = m_next->load_const_cached_vector_value(accessor, index);
        } else {
            value.data = accessor.const_vector_attribute(index);
        }
    }
    assert(value.data.size() == accessor.dimension());
    return value.data_as_const_map();
#endif
#endif
}

#if !defined(WMTK_FLUSH_ON_FAIL)
template <typename T>
auto AttributeScope<T>::scalar_attribute(AccessorBase<T>& accessor, int64_t index) -> T&
{
    return vector_attribute(accessor, index)(0);
}
#endif

template <typename T>
auto AttributeScope<T>::const_scalar_attribute(const AccessorBase<T>& accessor, int64_t index) const
    -> T
{
    return const_vector_attribute(accessor, index)(0);
}

template <typename T>
void AttributeScope<T>::flush(Attribute<T>& attr)
{
#if !defined(WMTK_FLUSH_ON_FAIL)
    if (m_next) {
        AttributeCache<T>::flush_to(*m_next);
    } else
#endif
    {
        AttributeCache<T>::flush_to(attr);
    }
}
template <typename T>
void AttributeScope<T>::flush_changes_to_vector(const Attribute<T>& attr, std::vector<T>& data)
{
#if !defined(WMTK_FLUSH_ON_FAIL)
    if (m_next) {
        m_next->flush_changes_to_vector(attr, data);
    }
    AttributeCache<T>::flush_to(attr, data);
#endif
}

template <typename T>
int64_t AttributeScope<T>::depth() const
{
    if (bool(m_next)) {
        return 1 + m_next->depth();
    } else {
        return 1;
    }
}

//template class AttributeScope<int64_t>;
//template class AttributeScope<double>;
//template class AttributeScope<char>;
//template class AttributeScope<Rational>;
} // namespace wmtk::attribute
