#pragma once
#include <Eigen/Core>
#include <map>
#include <memory>
#include "Attribute.hpp"


namespace wmtk::attribute {
template <typename T>
class Attribute;



// An attribute that that can track old state (transactions)
// A stack of transactions can be created/destroyed by push_scope/pop_scope
// 
template <typename T>
class CachingAttribute : public Attribute<T>
{
public:
    using Scalar = T;
    using BaseType = Attribute<T>;
    template <int D>
    using MapResult = typename BaseType::template MapResult<D>;
    template <int D>
    using ConstMapResult = typename BaseType::template ConstMapResult<D>;

    using BaseType::BaseType;

    CachingAttribute(const CachingAttribute&) = delete;
    CachingAttribute& operator=(const CachingAttribute&) = delete;
    CachingAttribute(CachingAttribute&&) = default;
    CachingAttribute& operator=(CachingAttribute&&) = default;

    // adds a scope for a new transaction
    void push_scope();
    // pops the current transaction
    // @param preserve_changes: if true the current state of the attribute is preserved, otherwise the last transaction level is popped
    void pop_scope(bool preserve_changes);








    /// checks that we are viewing the active state of the attribute
    bool at_current_scope() const;
    // index of the currently active transaction
    size_t current_transaction_index() const { return m_current_transaction_index; }
    // the total number of transactions
    int64_t transaction_depth() const;

    // whether any transactions exist
    bool has_transactions() const;


    /// default mutable vector access
    template <int D = Eigen::Dynamic>
    MapResult<D> vector_attribute(int64_t index);
    /// default immutable vector access

    template <int D = Eigen::Dynamic>
    ConstMapResult<D> const_vector_attribute(int64_t index) const;
    /// default mutable scalar access
    T& scalar_attribute(int64_t index);

    /// default immutable scalar access
    const T& const_scalar_attribute(int64_t index) const;

    template <int D = Eigen::Dynamic>
    /// specialized immutable scalar access useful for topological operations
    const T& const_vector_single_value(int64_t index, int8_t vector_index) const;

    void change_to_next_scope();
    void change_to_previous_scope();
    // go to the scope with active data
    void change_to_current_scope();
    void rollback_current_scope();





private:
    // raw access to a (strided) value in an attribute
    const T* get_value(int64_t index) const;
    // Appends data to the current cache/transaction
    template <typename Derived>
    void cache(int64_t index, const Eigen::MatrixBase<Derived>& value);
    // Appends scalar data to the current cache/transaction
    void cache(int64_t index, const T& value);
    void apply_last_scope();

public:// FUNCTIONS HERE ARE PUBLIC FOR UNIT TESTING. DO NOT USE!
    // purely used for debug printing out the ENTIRE attribute state
#if defined(WMTK_ENABLED_DEV_MODE)
    void print_state(std::string_view prefix) const;
#endif
    // resets the entire transaction stack. should only really be called in unit tests
    void reset();
    // clears the current active transaction, should only really be called in unit tests
    void clear();

    void apply_cache();
    // applyes to some other buffer that was passed in
    void apply_cache(std::vector<T>& other) const;

    const std::vector<T>& buffer() const { return m_buffer; }
    const std::vector<std::pair<size_t, size_t>>& indices() const { return m_indices; }

    // the starting index of each transaction
    const std::vector<size_t>& transaction_starts() const { return m_transaction_starts; }
    size_t buffer_end() const { return m_buffer_end; }
    size_t indices_end() const { return m_indices_end; }

    void emplace();
    void pop(bool preserve_changes);
    // go to the next most historic scope

    std::vector<std::pair<size_t, size_t>>::const_iterator transaction_start_begin(
        size_t scope_index) const;
    std::vector<std::pair<size_t, size_t>>::const_iterator final_transaction_end() const;

    std::vector<std::pair<size_t, size_t>>::const_reverse_iterator transaction_start_rend(
        size_t scope_index) const;
    std::vector<std::pair<size_t, size_t>>::const_reverse_iterator final_transaction_rbegin() const;

    void update_buffer_sizes_for_add(size_t data_size);
protected:
    std::vector<T> m_buffer = std::vector<T>(64);
    std::vector<std::pair<size_t, size_t>> m_indices = std::vector<std::pair<size_t, size_t>>(32);

    size_t m_current_transaction_index = 0;

    // the starting index of each transaction
    std::vector<size_t> m_transaction_starts;
    size_t m_buffer_end = 0;
    size_t m_indices_end = 0;
};


} // namespace wmtk::attribute


#if !defined(WMTK_ENABLED_DEV_MODE)
#include "CachingAttribute.hxx"
#endif
