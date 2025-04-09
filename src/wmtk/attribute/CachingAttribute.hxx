#pragma once
#include <wmtk/Types.hpp>
#include <wmtk/utils/Rational.hpp>
#include "CachingAttribute.hpp"

#if defined(WMTK_ENABLED_DEV_MODE)
#include <fmt/ranges.h>
#include <spdlog/spdlog.h>
#include <ranges>
#include <span>
#define WMTK_CACHING_ATTRIBUTE_INLINE


#else
#define WMTK_CACHING_ATTRIBUTE_INLINE inline
#endif

namespace wmtk::attribute {
#if defined(WMTK_ENABLED_DEV_MODE)
template <typename T>
void CachingAttribute<T>::print_state(std::string_view prefix) const
{
    if constexpr (std::is_same_v<T, Rational>) {
    } else if constexpr (std::is_same_v<T, char>) {
        auto toint = [](auto&& v) noexcept -> int64_t { return v; };
        spdlog::warn(
            "Attribute {}: [{}] on transaction {} of {}",
            BaseType::m_name,
            prefix,
            m_current_transaction_index,
            m_transaction_starts.size());
        spdlog::info("Data: {}", fmt::join(std::views::transform(BaseType::m_data, toint), ","));
        for (size_t j = 0; j < m_transaction_starts.size(); ++j) {
            size_t start = m_transaction_starts[j];
            size_t end;
            if (j == m_transaction_starts.size() - 1) {
                end = m_indices_end;
            } else {
                end = m_transaction_starts[j + 1];
            }
            spdlog::info("Detailing transaction {} with value indices {}->{}", j, start, end);

            for (size_t k = start; k < end; ++k) {
                const auto& [attr_index, table_index] = m_indices[k];
                spdlog::info(
                    "attr index {} has table index {} and value {}",
                    attr_index,
                    table_index,
                    fmt::join(
                        std::views::transform(
                            std::span(m_buffer.data() + table_index, BaseType::dimension()),
                            toint),
                        ","));
            }
        }
    }
}
#endif

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
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::rollback_current_scope()
{
    assert(has_transactions());
    assert(at_current_scope());
    apply_last_scope();
}

template <typename T>
template <int D>
WMTK_CACHING_ATTRIBUTE_INLINE auto CachingAttribute<T>::vector_attribute(int64_t index)
    -> MapResult<D>
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
WMTK_CACHING_ATTRIBUTE_INLINE auto CachingAttribute<T>::const_vector_attribute(int64_t index) const
    -> ConstMapResult<D>
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
WMTK_CACHING_ATTRIBUTE_INLINE auto CachingAttribute<T>::scalar_attribute(int64_t index) -> T&
{
    assert(at_current_scope());
    T& value = BaseType::scalar_attribute(index);
    if (has_transactions()) {
        cache(index, value);
    }
    return value;
}

template <typename T>
WMTK_CACHING_ATTRIBUTE_INLINE auto CachingAttribute<T>::const_scalar_attribute(int64_t index) const
    -> const T&
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
WMTK_CACHING_ATTRIBUTE_INLINE auto CachingAttribute<T>::const_vector_single_value(
    int64_t index,
    int8_t vector_index) const -> const T&
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
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::push_scope()
{
    emplace();
}
template <typename T>
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::pop_scope(bool preserve_changes)
{
    pop(preserve_changes);
}

template <typename T>
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::reset()
{
    m_transaction_starts.clear();
    change_to_current_scope();
    clear();
}

template <typename T>
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::clear()
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
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::update_buffer_sizes_for_add(
    size_t data_size)
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
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::cache(
    int64_t index,
    const Eigen::MatrixBase<Derived>& value)
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
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::cache(int64_t index, const T& value)
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
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::apply_cache()
{
    apply_cache(BaseType::m_data);
}

template <typename T>
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::apply_cache(std::vector<T>& other) const
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
WMTK_CACHING_ATTRIBUTE_INLINE auto CachingAttribute<T>::get_value(int64_t index) const -> const T*
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
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::emplace()
{
    assert(at_current_scope()); // must only be called on leaf node

    m_transaction_starts.emplace_back(m_indices_end);
    change_to_current_scope();
}
template <typename T>
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::pop(bool preserve_changes)
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
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::apply_last_scope()
{
    assert(at_current_scope());
    assert(has_transactions());
    apply_cache();
}

template <typename T>
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::change_to_previous_scope()
{
    // if the previous is a nullptr it's fine
    assert(!at_current_scope());
    m_current_transaction_index++;

}

template <typename T>
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::change_to_next_scope()
{
    if (at_current_scope()) {
        assert(has_transactions());
    }
    m_current_transaction_index--;
}
template <typename T>
WMTK_CACHING_ATTRIBUTE_INLINE void CachingAttribute<T>::change_to_current_scope()
{
    m_current_transaction_index = transaction_depth();
}
template <typename T>
WMTK_CACHING_ATTRIBUTE_INLINE bool CachingAttribute<T>::at_current_scope() const
{
    assert(m_current_transaction_index <= m_transaction_starts.size());
    return m_current_transaction_index == transaction_depth();
}

template <typename T>
WMTK_CACHING_ATTRIBUTE_INLINE int64_t CachingAttribute<T>::transaction_depth() const
{
    return m_transaction_starts.size();
}

template <typename T>
WMTK_CACHING_ATTRIBUTE_INLINE bool CachingAttribute<T>::has_transactions() const
{
    return !m_transaction_starts.empty();
}

} // namespace wmtk::attribute
#undef WMTK_CACHING_ATTRIBUTE_INLINE
