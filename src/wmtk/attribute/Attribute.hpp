#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>
#include <wmtk/utils/MerkleTreeInteriorNode.hpp>
#include "MapTypes.hpp"
#include "PerThreadAttributeScopeStacks.hpp"

namespace wmtk {
class MeshWriter;

namespace attribute {
template <typename T, int Dim>
class AccessorBase;

template <typename T>
class PerThreadAttributeScopeStacks;
template <typename T>
class AttributeScopeStack;
namespace internal {
template <typename T>
class AttributeMapCache;
}

/**
 * This class stores data of type T in a vector.
 * If multiple values should be hold per index, the data will be automatically linearized.
 * For example, per index we have a 3-dimensional vector. Then the data vector will contain:
 * [x0,y0,z0,x1,y1,z1,...]
 */
template <typename T>
class Attribute : public wmtk::utils::Hashable
{
public:
    template <int D = Eigen::Dynamic>
    using MapResult = internal::MapResult<T, D>;
    template <int D = Eigen::Dynamic>
    using ConstMapResult = internal::ConstMapResult<T, D>;


    // attribute directly hashes its "children" components so it overrides "child_hashes"
    std::map<std::string, size_t> child_hashes() const override;


    template <typename U, int D>
    friend class AccessorBase;
    friend class internal::AttributeMapCache<T>;
    void serialize(const std::string& name, const int dim, MeshWriter& writer) const;

    /**
     * @brief Initialize the attribute.
     *
     * @param dimension The dimension of the attribute, e.g. 3 for a 3d vector.
     * @param default_value A default value that is applied to every entry, also to new ones that
     * are added later.
     * @param size The number of expected indices. If size < 0 then the internal data is
     * not initialized.
     */
    Attribute(const std::string& name, int64_t dimension, T default_value = T(0), int64_t size = 0);

    Attribute(Attribute&& o);
    ~Attribute();
    Attribute& operator=(Attribute&& o);

    template <int D = Eigen::Dynamic>
    ConstMapResult<D> const_vector_attribute(const int64_t index) const;
    template <int D = Eigen::Dynamic>
    MapResult<D> vector_attribute(const int64_t index);
    template <int D = Eigen::Dynamic>
    MapResult<D> vector_attribute2(const int64_t index);

    T const_scalar_attribute(const int64_t index) const;
    T& scalar_attribute(const int64_t index);
    T const_scalar_attribute(const int64_t index, const int8_t offset) const;
    T& scalar_attribute(const int64_t index, const int8_t offset);

    /**
     * @brief Replace the internal data with `val`.
     */
    void set(std::vector<T> val);

    /**
     * @brief The total number of elements in a vector.
     * This is greater than the number of active values in the attribute, and the set of active
     * values is handled by a higher level abstraction
     */
    int64_t reserved_size() const;

    /**
     * @brief The number of values for each index.
     */
    int64_t dimension() const;
    void reserve(const int64_t size);

    /**
     * @brief returns the default value of this attribute
     */
    const T& default_value() const;

    bool operator==(const Attribute<T>& o) const;

    void push_scope();
    void pop_scope(bool apply_updates);
    void rollback_current_scope();

    const AttributeScopeStack<T>& get_local_scope_stack() const;
    AttributeScopeStack<T>& get_local_scope_stack();

    /**
     * @brief Consolidate the vector, using the new2old map m provided and resizing the vector to
     * m.size()
     */
    void consolidate(const std::vector<int64_t>& new2old);

    /**
     * @brief Applies the scalar old2new map to the indices in the attribute
     * This is commonly used after a consolidate to account for the change in global indices
     */
    void index_remap(const std::vector<T>& old2new);
    void index_remap(const std::vector<T>& old2new, const std::vector<Eigen::Index>& cols);

    /**
     * @brief Accesses the attribute using the specified vector as the underlying data
     * This is internally used by the single-arg const_vector_attribute and to help with
     * serialization
     */
    template <int D = Eigen::Dynamic>
    ConstMapResult<D> const_vector_attribute(const int64_t index, const std::vector<T>& data) const;

    /**
     * @brief Accesses the attribute using the specified vector as the underlying data
     * This is internally used by the single-arg const_vector_attribute and to help with
     * serialization. Start allows for assignment to buffers that dont' represent a 2d array
     */
    template <int D = Eigen::Dynamic>
    ConstMapResult<D> const_vector_attribute_from_start(const int64_t index, const std::vector<T>& data) const;
    /**
     * @brief Accesses the attribute using the specified vector as the underlying data
     * This is internally used by the single-arg vector_attribute and to help with serialization
     */
    template <int D = Eigen::Dynamic>
    MapResult<D> vector_attribute(const int64_t index, std::vector<T>& data) const;

    /**
     * @brief Accesses the attribute using the specified vector as the underlying data
     * This is internally used by the single-arg const_vector_attribute and to help with
     * serialization. Start allows for assignment to buffers that dont' represent a 2d array
     */
    template <int D = Eigen::Dynamic>
    MapResult<D> vector_attribute_from_start(const int64_t index, std::vector<T>& data) const;
    /**
     * @brief Accesses the attribute using the specified scalar as the underlying data
     * This is internally used by the single-arg const_scalar_attribute and to help with
     * serialization
     */
    T const_scalar_attribute(const int64_t index, const std::vector<T>& data) const;
    /**
     * @brief Accesses the attribute using the specified scalar as the underlying data
     * This is internally used by the single-arg scalar_attribute and to help with serialization
     */
    T& scalar_attribute(const int64_t index, std::vector<T>& data) const;

    /**
     * @brief Accesses the attribute using the specified scalar as the underlying data
     * This is internally used by the single-arg const_scalar_attribute and to help with
     * serialization
     */
    T const_scalar_attribute(const int64_t index, const int8_t offset, const std::vector<T>& data)
        const;
    /**
     * @brief Accesses the attribute using the specified scalar as the underlying data
     * This is internally used by the single-arg scalar_attribute and to help with serialization
     */
    T& scalar_attribute(const int64_t index, const int8_t offset, std::vector<T>& data) const;

    // computes the "reserved size" but using the passed in data
    int64_t reserved_size(const std::vector<T>& data) const;

private:
    std::vector<T> m_data;
    PerThreadAttributeScopeStacks<T> m_scope_stacks;
    int64_t m_dimension = -1;
    T m_default_value = T(0);

public:
    std::string m_name;
};

template <typename T>
template <int D>
inline auto Attribute<T>::const_vector_attribute(const int64_t index) const -> ConstMapResult<D>
{
    return const_vector_attribute<D>(index, m_data);
}

template <typename T>
template <int D>
inline auto Attribute<T>::const_vector_attribute(const int64_t index, const std::vector<T>& data)
    const -> ConstMapResult<D>
{
    assert(index < reserved_size(data));
    assert(data.size() % m_dimension == 0);
    const int64_t start = index * m_dimension;
    return const_vector_attribute_from_start<D>(start,data);
}
template <typename T>
template <int D>
inline auto Attribute<T>::const_vector_attribute_from_start(const int64_t start, const std::vector<T>& data)
    const -> ConstMapResult<D>
{
    assert(m_dimension > 0);
    if constexpr (D != Eigen::Dynamic) {
        assert(D == m_dimension);
    }
    ConstMapResult<D> R(data.data() + start, m_dimension);

    assert(R.size() == m_dimension);

    return R;
}


template <typename T>
template <int D>
inline auto Attribute<T>::vector_attribute(const int64_t index) -> MapResult<D>
{
    // return MapResult<D>(m_data.data(), m_dimension);
    return vector_attribute<D>(index, m_data);
}

template <typename T>
template <int D>
inline auto Attribute<T>::vector_attribute2(const int64_t index) -> MapResult<D>
{
    return MapResult<D>(m_data.data(), m_dimension);
    // return vector_attribute<D>(index, m_data);
}

template <typename T>
template <int D>
inline auto Attribute<T>::vector_attribute(const int64_t index, std::vector<T>& data) const
    -> MapResult<D>
{
    assert(index < reserved_size(data));
    assert(data.size() % m_dimension == 0);
    const int64_t start = index * m_dimension;
    return vector_attribute_from_start<D>(start,data);
}

template <typename T>
template <int D>
inline auto Attribute<T>::vector_attribute_from_start(const int64_t start, std::vector<T>& data) const
    -> MapResult<D>
{
    assert(m_dimension > 0);
    if constexpr (D != Eigen::Dynamic) {
        assert(D == m_dimension);
    }
    //assert(start < data.size());
    //assert(start + m_dimension < data.size());
    MapResult<D> R(data.data() + start, m_dimension);
    assert(R.size() == m_dimension);
    return R;
}

template <typename T>
inline T Attribute<T>::const_scalar_attribute(const int64_t index) const
{
    return const_scalar_attribute(index, m_data);
}
template <typename T>
inline T Attribute<T>::const_scalar_attribute(const int64_t index, const std::vector<T>& data) const
{
    assert(index < reserved_size(data));
    assert(m_dimension == 1);
    return data[index];
}

template <typename T>
inline T& Attribute<T>::scalar_attribute(const int64_t index)
{
    return scalar_attribute(index, m_data);
}
template <typename T>
inline T& Attribute<T>::scalar_attribute(const int64_t index, std::vector<T>& data) const
{
    assert(index < reserved_size(data));
    assert(m_dimension == 1);
    return data[index];
}

template <typename T>
inline T Attribute<T>::const_scalar_attribute(const int64_t index, const int8_t offset) const
{
    return const_scalar_attribute(index, offset, m_data);
}
template <typename T>
inline T Attribute<T>::const_scalar_attribute(
    const int64_t index,
    const int8_t offset,
    const std::vector<T>& data) const
{
    const int64_t idx = index * m_dimension + offset;
    assert(index < reserved_size(data));
    return data[idx];
}

template <typename T>
inline T& Attribute<T>::scalar_attribute(const int64_t index, const int8_t offset)
{
    return scalar_attribute(index, offset, m_data);
}
template <typename T>
inline T&
Attribute<T>::scalar_attribute(const int64_t index, const int8_t offset, std::vector<T>& data) const
{
    const int64_t idx = index * m_dimension + offset;
    assert(index < reserved_size(data));
    return data[idx];
}


template <typename T>
inline int64_t Attribute<T>::dimension() const
{
    return m_dimension;
}

template <typename T>
inline const T& Attribute<T>::default_value() const
{
    return m_default_value;
}

template <typename T>
inline const AttributeScopeStack<T>& Attribute<T>::get_local_scope_stack() const
{
    return m_scope_stacks.local();
}
template <typename T>
inline AttributeScopeStack<T>& Attribute<T>::get_local_scope_stack()
{
    return m_scope_stacks.local();
}

template <typename T>
inline void Attribute<T>::push_scope()
{
    m_scope_stacks.local().emplace();
}
template <typename T>
inline void Attribute<T>::pop_scope(bool apply_updates)
{
    m_scope_stacks.local().pop(*this, apply_updates);
}

template <typename T>
inline void Attribute<T>::rollback_current_scope()
{
    m_scope_stacks.local().rollback_current_scope(*this);
}

} // namespace attribute
} // namespace wmtk
#include "AccessorBase.hpp"
#include "AttributeCache.hpp"
