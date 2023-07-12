#include "AttributeCache.hpp"

template <typename T>
struct AttributeCache<T>::Data
{
    template <typename Derived>
    Data(const Eigen::MatrixBase<Derived>& a, bool d = false)
        : data(a)
        , dirty(d)
    {}
    Data(bool d = false)
        : dirty(d)
    {}
    Eigen::Matrix<T, Eigen::Dynamic, 1> data;
    bool dirty = false;
};


template <typename T>
auto AttributeCache<T>::load_it(long index) const -> std::pair<typename DataStorage::iterator, bool>
{
    if (auto it = m_data.find(index); it != m_data.end()) {
        return {it, false};
    } else {
        return m_data.try_emplace(index, false);
    }
}

template <typename T>
auto AttributeCache<T>::load_it(const AccessorBase<T>& accessor, long index) const ->
    typename DataStorage::iterator
{
    auto [it, was_inserted] = load_it(index);
    if (was_inserted) {
        it.second.data = load_cached_vector_value(accessor, index);
    }
    return it;
}


template <typename T>
auto vector_attribute(const AccessorBase<T>& accessor, AccessorAccessMode mode, long index) const
    -> MapResult
{
    switch (mode) {
    case AccessorAccessMode::Buffered: {
        auto it = load_it(accessor, mode);
        return MapResultT<IsConst>(dat.data(), dat.size());
    }
    case AccessorAccessMode::Immediate:
    default: {
        return load_cached_vector_value(accessor, index);
    }
    }
}

template <typename T>
auto AttributeCache<T>::const_vector_attribute(const AccessorBase<T>& accessor, AccessorAccessMode mode, long index)
    const -> ConstMapResult
{
    switch (mode) {
    case AccessorAccessMode::Buffered: {
        auto it = load_it(accessor, mode);
        return ConstMapResult<IsConst>(dat.data(), dat.size());
    }
    case AccessorAccessMode::Immediate:
    default: {
        return load_cached_vector_value(accessor, index);
    }
    }
}

template <typename T>
auto AttributeCache<T>::scalar_attribute(AccessorBase<T>& accessor, AccessorAccessMode mode, long index) -> T&
{
    return vector_attribute(accessor, mode, index)(0);
}

template <typename T>
auto AttributeCache<T>::const_scalar_attribute(
    const AccessorBase<T>& accessor,
    AccessorAccessMode mode,
    long index) const->T
{
    return const_vector_attribute(accessor, mode, index)(0);
}

void clear()
{
    m_data.clear();
}

template <typename T>
auto AttributeCache<T>::load_const_cached_scalar_value(const AccessorBase<T>& accessor, long index) -> T
{
    if (auto it = m_data.find(index); it != m_data.end()) {
        const auto& dat = data.data;
        assert(dat.size() == 1);
        return dat(0);
    } else if (m_parent_cache) {
        return m_parent_cache->load_const_cached_scalar_value(accessor);
    } else {
        return accessor.vector_attribute(index);
    }
}

template <typename T>
auto AttributeCache<T>::load_cached_scalar_value(AccessorBase<T>& accessor, long index) -> T&
{
    if (auto it = m_data.find(index); it != m_data.end()) {
        auto& dat = data.data;
        assert(dat.size() == 1);
        return dat(0);
    } else if (m_parent_cache) {
        return m_parent_cache->load_cached_vector_value(accessor);
    } else {
        return accessor.vector_attribute(index);
    }
}


template <typename T>
auto AttributeCache<T>::load_cached_vector_value(AccessorBase<T>& accessor, long index) -> MapResult
{
    if (auto it = m_data.find(index); it != m_data.end()) {
        auto& dat = data.data;
        return MapResult(dat.data(), dat.size());
    } else if (m_parent_cache) {
        return m_parent_cache->load_cached_vector_value(accessor);
    } else {
        return accessor.vector_attribute(index);
    }
}
auto AttributeCache<T>::load_const_cached_vector_value(const AccessorBase<T>& accessor, long index)
    -> ConstMapResult
{
    if (auto it = m_data.find(index); it != m_data.end()) {
        auto& dat = data.data;
        return ConstMapResult(dat.data(), dat.size());
    } else if (m_parent_cache) {
        return m_parent_cache->load_cached_vector_value(accessor);
    } else {
        return accessor.const_vector_attribute(index);
    }
}
template <typename T>
void AttributeCache<T>::flush(AccessorBase<T>& accessor) const
{
    if (m_parent_cache) {
        auto& parent_data = m_parent_cache->m_data;

        for (auto& [index, data] : m_data) {
            if (data.dirty) {
                parent_data[index] = data;
            }
            data.dirty = false;
        }
    } else {
        for (const auto& [index, data] : m_data) {
            if (data.dirty) {
                accessor.vector_attribute(index) = data.data;
            }
            data.dirty = false;
        }
    }
}
}
