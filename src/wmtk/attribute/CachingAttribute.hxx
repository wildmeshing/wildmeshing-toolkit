#pragma once
#include <wmtk/Types.hpp>
#include <wmtk/utils/Rational.hpp>
#include "CachingAttribute.hpp"

namespace wmtk::attribute {

template <typename T>
auto CachingAttribute<T>::transaction_start_begin(size_t scope_index) const
    -> std::vector<std::pair<size_t, size_t>>::const_iterator
{
    return m_indices.begin() + m_transaction_starts[scope_index];
}
template <typename T>
auto CachingAttribute<T>::final_transaction_end() const
    -> std::vector<std::pair<size_t, size_t>>::const_iterator
{
    return m_indices.begin() + m_indices_end;
}

template <typename T>
auto CachingAttribute<T>::transaction_start_rend(size_t scope_index) const
    -> std::vector<std::pair<size_t, size_t>>::const_reverse_iterator
{
    return std::reverse_iterator(transaction_start_begin(scope_index));
}
template <typename T>
auto CachingAttribute<T>::final_transaction_rbegin() const
    -> std::vector<std::pair<size_t, size_t>>::const_reverse_iterator
{
    return std::reverse_iterator(final_transaction_end());
}

template <typename T>
inline void CachingAttribute<T>::rollback_current_scope()
{
    assert(has_transactions());
    assert(at_current_scope());
    apply_last_scope();
}

template <typename T>
template <int D>
inline auto CachingAttribute<T>::vector_attribute(int64_t index) -> MapResult<D>
{
    assert(at_current_scope());

    auto data = BaseType::template vector_attribute<D>(index);
    assert(data.cols() == 1);
    if constexpr (D != Eigen::Dynamic) {
        assert(data.size() == D);
    }
    // we are typically only going to write when caching is enabled so better to optimize for this
    if (has_transactions()) {
        cache(index, data);
    }
    return data;
}


template <typename T>
template <int D>
inline auto CachingAttribute<T>::const_vector_attribute(int64_t index) const -> ConstMapResult<D>
{
    if (!at_current_scope()) {
        assert(m_current_transaction_index < m_transaction_starts.size());

        const T* ptr = get_value(index);
        if (ptr != nullptr) {
            const int dim = BaseType::dimension();
            auto dat = ConstMapResult<D>(ptr, dim);
            return dat;
        }
    }
    return BaseType::template const_vector_attribute<D>(index);
}

template <typename T>
inline auto CachingAttribute<T>::scalar_attribute(int64_t index) -> T&
{
    assert(at_current_scope());
    T& value = BaseType::scalar_attribute(index);
    if (has_transactions()) {
        cache(index, value);
    }
    return value;
}

template <typename T>
inline auto CachingAttribute<T>::const_scalar_attribute(int64_t index) const -> const T&
{
    if (!at_current_scope()) {
        assert(m_current_transaction_index < m_transaction_starts.size());

        const T* ptr = get_value(index);
        if (ptr != nullptr) {
            return *ptr;
        }
    }

    return BaseType::const_scalar_attribute(index);
}
template <typename T>
template <int D>
inline auto CachingAttribute<T>::const_vector_single_value(int64_t index, int8_t vector_index) const
    -> const T&
{
    if (!at_current_scope()) {
        assert(m_current_transaction_index < m_transaction_starts.size());

        const T* ptr = get_value(index);
        if (ptr != nullptr) {
            const int dim = BaseType::dimension();
            assert(vector_index < dim);
            return ptr[vector_index];
        }
    }
    return BaseType::const_vector_single_value(index, vector_index);
}

//=======================================================
// Scope members
//=======================================================
template <typename T>
inline void CachingAttribute<T>::push_scope()
{
    emplace();
}
template <typename T>
inline void CachingAttribute<T>::pop_scope(bool preserve_changes)
{
    pop(preserve_changes);
}

template <typename T>
inline void CachingAttribute<T>::reset()
{
    m_transaction_starts.clear();
    change_to_current_scope();
    clear();
}

template <typename T>
inline void CachingAttribute<T>::clear()
{
    // clearing a scope means nothing?
    if (!has_transactions()) {
        m_indices_end = 0;
        m_buffer_end = 0;
    } else {
        m_indices_end = m_transaction_starts.back();
        m_buffer_end = m_indices[m_indices_end].second;
    }
}

template <typename T>
inline void CachingAttribute<T>::update_buffer_sizes_for_add(size_t data_size)
{
    // check sizes
    if (m_indices_end + 1 >= m_indices.size()) {
        assert(m_indices.size() > 2);
        m_indices.resize(m_indices.size() * 1.75);
    }

    if (m_buffer_end + data_size >= m_buffer.size()) {
        assert(m_buffer.size() > 2);
        m_buffer.resize(m_buffer.size() * 1.75);
    }
}

template <typename T>
template <typename Derived>
inline void CachingAttribute<T>::cache(int64_t index, const Eigen::MatrixBase<Derived>& value)
{
    // basically try_emplace but optimizes to avoid accessing the pointed-to value

    size_t dim = value.size();


    update_buffer_sizes_for_add(dim);

    // assert(old_size % dim == 0);
    //assert(old_size / value.size() == m_indices.size());
    // m_indices.emplace_back(index, m_indices.size());
    m_indices[m_indices_end] = {index, m_buffer_end};


    assert(m_buffer.size() >= m_buffer_end + dim);
    // m_buffer.resize(m_buffer_end + dim);
    std::copy(value.begin(), value.end(), m_buffer.begin() + m_buffer_end);
    m_buffer_end += dim;
    m_indices_end++;
    assert(m_buffer_end == m_indices_end * dim);
}

template <typename T>
inline void CachingAttribute<T>::cache(int64_t index, const T& value)
{
    update_buffer_sizes_for_add(1);
    // assert(m_buffer.size() == m_indices.size());
    m_indices[m_indices_end] = {index, m_buffer_end};
    m_buffer[m_buffer_end] = value;

    m_buffer_end++;
    m_indices_end++;
    assert(m_buffer_end == m_indices_end);
}


template <typename T>
inline void CachingAttribute<T>::apply_cache()
{
    apply_cache(BaseType::m_data);
}

template <typename T>
inline void CachingAttribute<T>::apply_cache(std::vector<T>& other) const
{
    assert(at_current_scope());
    for (auto it = final_transaction_rbegin();
         it != transaction_start_rend(current_transaction_index() - 1);
         ++it) {
        const auto& [global, local] = *it;
        auto a = BaseType::vector_attribute(global, other);
        auto b = BaseType::const_vector_attribute_without_stride(local, m_buffer);
        a = b;
    }
}

template <typename T>
inline auto CachingAttribute<T>::get_value(int64_t index) const -> const T*
{
    for (auto it = transaction_start_begin(current_transaction_index());
         it != final_transaction_end();
         ++it) {
        const auto& [global_index, local_index] = *it;
        if (global_index == index) {
            const T* ptr = m_buffer.data() + local_index;
            return ptr;
        }
    }
    return nullptr;
}

template <typename T>
inline void CachingAttribute<T>::emplace()
{
    assert(at_current_scope()); // must only be called on leaf node

    m_transaction_starts.emplace_back(m_indices_end);
    change_to_current_scope();
}
template <typename T>
inline void CachingAttribute<T>::pop(bool preserve_changes)
{
    assert(at_current_scope()); // must only be called on leaf node
    // TODO consider re-enabling
    if (!preserve_changes) {
        apply_cache();
    }


    m_transaction_starts.pop_back();

    change_to_current_scope();
    if (!has_transactions()) {
        m_indices_end = 0;
        m_buffer_end = 0;
    }
}


template <typename T>
inline void CachingAttribute<T>::apply_last_scope()
{
    assert(at_current_scope());
    assert(has_transactions());
    apply_cache();
}

template <typename T>
inline void CachingAttribute<T>::change_to_previous_scope()
{
    // if the previous is a nullptr it's fine
    assert(!at_current_scope());
    m_current_transaction_index++;
}

template <typename T>
inline void CachingAttribute<T>::change_to_next_scope()
{
    if (at_current_scope()) {
        assert(has_transactions());
    }
    m_current_transaction_index--;
}
template <typename T>
inline void CachingAttribute<T>::change_to_current_scope()
{
    m_current_transaction_index = transaction_depth();
}
template <typename T>
inline bool CachingAttribute<T>::at_current_scope() const
{
    assert(m_current_transaction_index <= m_transaction_starts.size());
    return m_current_transaction_index == transaction_depth();
}

template <typename T>
inline int64_t CachingAttribute<T>::transaction_depth() const
{
    return m_transaction_starts.size();
}

template <typename T>
inline bool CachingAttribute<T>::has_transactions() const
{
    return !m_transaction_starts.empty();
}

} // namespace wmtk::attribute
