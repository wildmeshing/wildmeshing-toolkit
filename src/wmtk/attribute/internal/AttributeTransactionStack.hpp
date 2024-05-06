#pragma once
#include <Eigen/Core>
#include <map>
#include <memory>
#include <wmtk/attribute/Attribute.hpp>
#include "MapTypes.hpp"


namespace wmtk::attribute {
template <typename T>
class Attribute;
template <typename T, int Dim>
class AccessorBase;
namespace internal {
template <typename T>
class AttributeTransactionStack
{
public:
    template <int D>
    using MapResult = internal::MapResult<T, D>;
    template <int D>
    using ConstMapResult = internal::ConstMapResult<T, D>;


    AttributeTransactionStack();
    ~AttributeTransactionStack();
    AttributeTransactionStack(const AttributeTransactionStack&) = delete;
    AttributeTransactionStack& operator=(const AttributeTransactionStack&) = delete;
    AttributeTransactionStack(AttributeTransactionStack&&) = default;
    AttributeTransactionStack& operator=(AttributeTransactionStack&&) = default;


    template <typename Derived>
    void try_caching(int64_t index, const Eigen::MatrixBase<Derived>& value);
    void try_caching(int64_t index, const T& value);


    const T* get_value(int64_t index) const;

    // clears a stack
    void clear();
    int64_t size() const;

    void apply_to(Attribute<T>& attribute) const;

    // applyes to some other buffer that was passed in
    void apply_to(const Attribute<T>& attribute, std::vector<T>& other) const;

    const std::vector<T>& buffer() const { return m_buffer; }
    const std::vector<std::pair<size_t, size_t>>& indices() const { return m_indices; }


    size_t current_transaction_index() { return m_current_transaction_index; }

    // the starting index of each transaction
    const std::vector<size_t>& transaction_starts() const { return m_transaction_starts; }
    size_t buffer_end() const { return m_buffer_end; }
    size_t indices_end() const { return m_indices_end; }

    // OLD API
    //==========


    bool empty() const;


    bool writing_enabled() const;


    /// default mutable vector access
    template <
        int D = Eigen::Dynamic,
        int D2 = Eigen::Dynamic,
        int TrueD = D == Eigen::Dynamic ? D2 : D>
    MapResult<TrueD> vector_attribute(AccessorBase<T, D2>& accessor, int64_t index);
    /// default immutable vector access

    template <
        int D = Eigen::Dynamic,
        int D2 = Eigen::Dynamic,
        int TrueD = D == Eigen::Dynamic ? D2 : D>
    ConstMapResult<TrueD> const_vector_attribute(const AccessorBase<T, D2>& accessor, int64_t index)
        const;
    /// default mutable scalar access
    template <int D2>
    T& scalar_attribute(AccessorBase<T, D2>& accessor, int64_t index);

    /// default immutable scalar access
    template <int D = Eigen::Dynamic>
    T const_scalar_attribute(const AccessorBase<T, D>& accessor, int64_t index) const;

    template <int D = Eigen::Dynamic>
    /// specialized immutable scalar access useful for topological operations
    T const_scalar_attribute(const AccessorBase<T, D>& accessor, int64_t index, int8_t offset)
        const;

    void emplace();
    void pop(Attribute<T>& attribute, bool preserve_changes);
    // go to the next most historic scope
    void change_to_next_scope();
    void change_to_previous_scope();
    // go to the scope with active data
    void change_to_current_scope();
    void rollback_current_scope(Attribute<T>& attr);

    /// checks that we are viewing the active state of the attribute
    bool at_current_scope() const;


    size_t current_transaction_index() const { return m_current_transaction_index; }

    std::vector<std::pair<size_t, size_t>>::const_iterator transaction_start_begin(
        size_t scope_index) const;
    std::vector<std::pair<size_t, size_t>>::const_iterator final_transaction_end() const;

    std::vector<std::pair<size_t, size_t>>::const_reverse_iterator transaction_start_rend(
        size_t scope_index) const;
    std::vector<std::pair<size_t, size_t>>::const_reverse_iterator final_transaction_rbegin() const;

    void update_buffer_sizes_for_add(size_t data_size);


private:
    void apply_last_scope(Attribute<T>& attr);

protected:
    std::vector<T> m_buffer;
    std::vector<std::pair<size_t, size_t>> m_indices;

    size_t m_current_transaction_index = 0;

    // the starting index of each transaction
    std::vector<size_t> m_transaction_starts;
    size_t m_buffer_end;
    size_t m_indices_end;
};


template <typename T>
auto AttributeTransactionStack<T>::transaction_start_begin(size_t scope_index) const

    -> std::vector<std::pair<size_t, size_t>>::const_iterator
{
    return m_indices.begin() + m_transaction_starts[scope_index];
}
template <typename T>
auto AttributeTransactionStack<T>::final_transaction_end() const
    -> std::vector<std::pair<size_t, size_t>>::const_iterator

{
    return m_indices.begin() + m_indices_end;
}

template <typename T>
auto AttributeTransactionStack<T>::transaction_start_rend(size_t scope_index) const
    -> std::vector<std::pair<size_t, size_t>>::const_reverse_iterator
{
    return std::reverse_iterator(transaction_start_begin(scope_index));
}
template <typename T>
auto AttributeTransactionStack<T>::final_transaction_rbegin() const
    -> std::vector<std::pair<size_t, size_t>>::const_reverse_iterator
{
    return std::reverse_iterator(final_transaction_end());
}

template <typename T>
inline int64_t AttributeTransactionStack<T>::size() const
{
    return m_transaction_starts.size();
}

template <typename T>
inline void AttributeTransactionStack<T>::rollback_current_scope(Attribute<T>& attr)
{
    assert(!empty());
    assert(at_current_scope());
    apply_last_scope(attr);
}

template <typename T>
template <int D, int D2, int TrueD>
inline auto AttributeTransactionStack<T>::vector_attribute(
    AccessorBase<T, D2>& accessor,
    int64_t index) -> MapResult<TrueD>
{
    assert(writing_enabled());

    static_assert(D == Eigen::Dynamic || D2 == Eigen::Dynamic || D == D2);
    auto data = accessor.template vector_attribute<TrueD>(index);
    assert(data.cols() == 1);
    if constexpr (D != Eigen::Dynamic) {
        assert(data.size() == D);
    }
    if constexpr (D2 != Eigen::Dynamic) {
        assert(data.size() == D2);
    }
    // we are typically only going to write when caching is enabled so better to optimize for this
    if (!empty()) {
        // assert(m_scopes.back() == *m_back);
        try_caching(index, data);
        // m_back->try_caching(index,data);
    }
    return data;
}

template <typename T>
template <int D, int D2, int TrueD>
inline auto AttributeTransactionStack<T>::const_vector_attribute(
    const AccessorBase<T, D2>& accessor,
    int64_t index) const -> ConstMapResult<TrueD>
{
    static_assert(D == Eigen::Dynamic || D2 == Eigen::Dynamic || D == D2);
    if (!at_current_scope()) {
        assert(m_current_transaction_index < m_transaction_starts.size());

        const T* ptr = get_value(index);
        if (ptr != nullptr) {
            const int dim = accessor.dimension();
            auto dat = ConstMapResult<TrueD>(ptr, dim);
            return dat;
        }
    }
    return accessor.template const_vector_attribute<TrueD>(index);
}

template <typename T>
template <int D2>
inline auto AttributeTransactionStack<T>::scalar_attribute(
    AccessorBase<T, D2>& accessor,
    int64_t index) -> T&
{
    assert(writing_enabled());
    T& value = accessor.scalar_attribute(index);
    if (!empty()) {
        try_caching(index, value);
        // m_active->try_caching(index, value);
    }
    return value;
}

template <typename T>
template <int D2>
inline auto AttributeTransactionStack<T>::const_scalar_attribute(
    const AccessorBase<T, D2>& accessor,
    int64_t index) const -> T
{
    return const_vector_attribute<1>(accessor, index)(0);
}
template <typename T>
template <int D2>
inline auto AttributeTransactionStack<T>::const_scalar_attribute(
    const AccessorBase<T, D2>& accessor,
    int64_t index,
    int8_t offset) const -> T
{
    if (!at_current_scope()) {
        return const_vector_attribute<D2>(accessor, index)(offset);
    } else {
        return accessor.const_scalar_attribute(index, offset);
    }
}
} // namespace internal
} // namespace wmtk::attribute

#include "AttributeTransactionStack.hxx"
