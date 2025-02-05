#pragma once
#include <Eigen/Core>
#include <map>
#include <memory>
#include "internal/VectorTypes.hpp"
#include "Attribute.hpp"


namespace wmtk::attribute {
template <typename T>
class Attribute;
template <typename T>
class CachingAttribute : public Attribute<T>
{
public:
    using BaseType = Attribute<T>;
    template <int D>
    using MapResult = typename BaseType::MapResult<D>;
    template <int D>
    using ConstMapResult = typename BaseType::ConstMapResult<D>;

    using BaseType::BaseType;

    CachingAttribute(const CachingAttribute&) = delete;
    CachingAttribute& operator=(const CachingAttribute&) = delete;
    CachingAttribute(CachingAttribute&&) = default;
    CachingAttribute& operator=(CachingAttribute&&) = default;

    void push_scope();
    void pop_scope(bool apply_updates);


    template <typename Derived>
    void try_caching(int64_t index, const Eigen::MatrixBase<Derived>& value);
    void try_caching(int64_t index, const T& value);


    const T* get_value(int64_t index) const;

    // clears the current active transaction
    void clear();
    // resets the entire transaction stack. should only really be called in unit tests 
    void reset();
    int64_t size() const;

    void apply_to() ;
    // applyes to some other buffer that was passed in
    void apply_to(std::vector<T>& other) const;

    const std::vector<T>& buffer() const { return m_buffer; }
    const std::vector<std::pair<size_t, size_t>>& indices() const { return m_indices; }


    size_t current_transaction_index() { return m_current_transaction_index; }

    // the starting index of each transaction
    const std::vector<size_t>& transaction_starts() const { return m_transaction_starts; }
    size_t buffer_end() const { return m_buffer_end; }
    size_t indices_end() const { return m_indices_end; }

    // OLD API
    //==========


    bool transactions_empty() const;


    bool writing_enabled() const;


    /// default mutable vector access
    template <int D = Eigen::Dynamic>
    MapResult<D> vector_attribute(int64_t index);
    /// default immutable vector access

    template <int D = Eigen::Dynamic>
    ConstMapResult<D> const_vector_attribute(int64_t index) const;
    /// default mutable scalar access
    T& scalar_attribute(int64_t index);

    /// default immutable scalar access
    T const_scalar_attribute(int64_t index) const;

    template <int D = Eigen::Dynamic>
    /// specialized immutable scalar access useful for topological operations
    T const_vector_single_value(int64_t index, int8_t vector_index) const;

    void emplace();
    void pop(bool preserve_changes);
    // go to the next most historic scope
    void change_to_next_scope();
    void change_to_previous_scope();
    // go to the scope with active data
    void change_to_current_scope();
    void rollback_current_scope();

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
    void apply_last_scope();

protected:
    std::vector<T> m_buffer = std::vector<T>(64);
    std::vector<std::pair<size_t, size_t>> m_indices = std::vector<std::pair<size_t, size_t>>(32);

    size_t m_current_transaction_index = 0;

    // the starting index of each transaction
    std::vector<size_t> m_transaction_starts;
    size_t m_buffer_end = 0;
    size_t m_indices_end = 0;
};


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
inline int64_t CachingAttribute<T>::size() const
{
    return m_transaction_starts.size();
}

template <typename T>
inline void CachingAttribute<T>::rollback_current_scope()
{
    assert(!transactions_empty());
    assert(at_current_scope());
    apply_last_scope();
}

template <typename T>
template <int D>
inline auto CachingAttribute<T>::vector_attribute(int64_t index) -> MapResult<D>
{
    assert(writing_enabled());

    auto data = BaseType::template vector_attribute<D>(index);
    assert(data.cols() == 1);
    if constexpr (D != Eigen::Dynamic) {
        assert(data.size() == D);
    }
    // we are typically only going to write when caching is enabled so better to optimize for this
    if (!transactions_empty()) {
        try_caching(index, data);
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
    assert(writing_enabled());
    T& value = BaseType::scalar_attribute(index);
    if (!transactions_empty()) {
        try_caching(index, value);
    }
    return value;
}

template <typename T>
inline auto CachingAttribute<T>::const_scalar_attribute(int64_t index) const -> T
{
    return const_vector_attribute<1>(index)(0);
}
template <typename T>
template <int D>
inline auto CachingAttribute<T>::const_vector_single_value(int64_t index, int8_t vector_index) const
    -> T
{
    if (!at_current_scope()) {
        return const_vector_attribute<D>(index)(vector_index);
    } else {
        return BaseType::const_vector_single_value(index, vector_index);
    }
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
inline void CachingAttribute<T>::pop_scope(bool apply_updates)
{
    pop(apply_updates);
}

} // namespace wmtk::attribute


 #include "CachingAttribute.hxx"
