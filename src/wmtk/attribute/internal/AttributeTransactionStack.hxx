
#include <wmtk/Types.hpp>
#include <wmtk/attribute/AccessorBase.hpp>
#include <wmtk/utils/Rational.hpp>
#include "AttributeTransactionStack.hpp"


namespace wmtk::attribute::internal {


template <typename T>
inline AttributeTransactionStack<T>::AttributeTransactionStack()
    : m_buffer(64)
    , m_indices(32)
    , m_buffer_end(0)
    , m_indices_end(0)
{}
template <typename T>
inline AttributeTransactionStack<T>::~AttributeTransactionStack() = default;


template <typename T>
inline void AttributeTransactionStack<T>::clear()
{
    // m_indices_end = m_transaction_starts.back();
    // m_buffer_end = m_indices[m_indices_end].second;
}

template <typename T>
inline void AttributeTransactionStack<T>::update_buffer_sizes_for_add(size_t data_size)
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
inline void AttributeTransactionStack<T>::try_caching(
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
inline void AttributeTransactionStack<T>::try_caching(int64_t index, const T& value)
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
inline void AttributeTransactionStack<T>::apply_to(Attribute<T>& attribute) const
{
    apply_to(attribute, attribute.m_data);
}

template <typename T>
inline void AttributeTransactionStack<T>::apply_to(
    const Attribute<T>& attribute,
    std::vector<T>& other) const
{
    assert(at_current_scope());
    for (auto it = final_transaction_rbegin();
         it != transaction_start_rend(current_transaction_index() - 1);
         ++it) {
        const auto& [global, local] = *it;
        auto a = attribute.vector_attribute(global, other);
        auto b = attribute.const_vector_attribute_from_start(local, m_buffer);
        a = b;
    }
}

template <typename T>
inline auto AttributeTransactionStack<T>::get_value(int64_t index) const -> const T*
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
inline void AttributeTransactionStack<T>::emplace()
{
    assert(at_current_scope()); // must only be called on leaf node

    m_transaction_starts.emplace_back(m_indices_end);
    change_to_current_scope();
}
template <typename T>
inline void AttributeTransactionStack<T>::pop(Attribute<T>& attribute, bool preserve_changes)
{
    assert(at_current_scope()); // must only be called on leaf node
    // TODO consider re-enabling
    if (preserve_changes) {
        clear();
    } else {
        apply_to(attribute);
    }


    m_transaction_starts.pop_back();

    change_to_current_scope();
    if (empty()) {
        m_indices_end = 0;
        m_buffer_end = 0;
    }
}


template <typename T>
inline bool AttributeTransactionStack<T>::empty() const
{
    return m_transaction_starts.empty();
}


template <typename T>
inline void AttributeTransactionStack<T>::apply_last_scope(Attribute<T>& attr)
{
    assert(at_current_scope());
    assert(!empty());
    apply_to(attr);
}

template <typename T>
inline void AttributeTransactionStack<T>::change_to_previous_scope()
{
    // if the previous is a nullptr it's fine
    assert(!at_current_scope());
    m_current_transaction_index++;
}

template <typename T>
inline void AttributeTransactionStack<T>::change_to_next_scope()
{
    if (at_current_scope()) {
        assert(!empty());
    }
    m_current_transaction_index--;
}
template <typename T>
inline void AttributeTransactionStack<T>::change_to_current_scope()
{
    m_current_transaction_index = m_transaction_starts.size();
}
template <typename T>
inline bool AttributeTransactionStack<T>::at_current_scope() const
{
    assert(m_current_transaction_index <= m_transaction_starts.size());
    return m_current_transaction_index == m_transaction_starts.size();
}
template <typename T>
inline bool AttributeTransactionStack<T>::writing_enabled() const
{
    return at_current_scope();
}

} // namespace wmtk::attribute::internal
