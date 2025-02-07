#pragma once

#include <Eigen/Core>
#include <vector>
#include <wmtk/utils/MerkleTreeInteriorNode.hpp>
#include "MapTypes.hpp"

namespace wmtk {
class MeshWriter;

namespace attribute {
template <typename T, int Dim>
class AccessorBase;


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
    // typedefs
    template <int D = Eigen::Dynamic>
    using MapResult = MapResult<T, D>;
    template <int D = Eigen::Dynamic>
    using ConstMapResult = ConstMapResult<T, D>;


    // attribute directly hashes its "children" components so it overrides "child_hashes"
    std::map<std::string, size_t> child_hashes() const override;


    template <typename U, int D>
    friend class AccessorBase;
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
    Attribute& operator=(Attribute&& o);

    ~Attribute() override;

    /// Access the value of an attribute at a particular index. If the dimension of the attribute is known at compile time then the template parameter should be elided to improve performance.
    template <int D = Eigen::Dynamic>
    ConstMapResult<D> const_vector_attribute(const int64_t index) const;
    /// Access the value of an attribute at a particular index. If the dimension of the attribute is known at compile time then the template parameter should be elided to improve performance.
    template <int D = Eigen::Dynamic>
    MapResult<D> vector_attribute(const int64_t index);

    /// Access the value of a scalar attribute
    const T& const_scalar_attribute(const int64_t index) const;
    /// Access the value of a scalar attribute. Assignment to the returned value will change the value
    T& scalar_attribute(const int64_t index);
    /// Access a single entry in a vector attribute. TODO: this might not actually be more performant than
    template <int D = Eigen::Dynamic>
    const T& const_vector_single_value(const int64_t index, const int8_t vector_index) const;
    /// Access to a single a single value of a scalr attribute
    template <int D = Eigen::Dynamic>
    T& vector_single_value(const int64_t index, const int8_t vector_index);

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
    ConstMapResult<D> const_vector_attribute_without_stride(
        int64_t index,
        const std::vector<T>& data) const;
    /**
     * @brief Accesses the attribute using the specified vector as the underlying data
     * This is internally used by the single-arg vector_attribute and to help with serialization
     */
    template <int D = Eigen::Dynamic>
    MapResult<D> vector_attribute(int64_t index, std::vector<T>& data) const;

    /**
     * @brief Accesses the attribute using the specified vector as the underlying data
     * This is internally used by the single-arg const_vector_attribute and to help with
     * serialization. Start allows for assignment to buffers that dont' represent a 2d array
     */
    template <int D = Eigen::Dynamic>
    MapResult<D> vector_attribute_without_stride(int64_t index, std::vector<T>& data) const;
    /**
     * @brief Accesses the attribute using the specified scalar as the underlying data
     * This is internally used by the single-arg const_scalar_attribute and to help with
     * serialization
     */
    const T& const_scalar_attribute(int64_t index, const std::vector<T>& data) const;
    /**
     * @brief Accesses the attribute using the specified scalar as the underlying data
     * This is internally used by the single-arg scalar_attribute and to help with serialization
     */
    T& scalar_attribute(int64_t index, std::vector<T>& data) const;

    /**
     * @brief Accesses the attribute using the specified scalar as the underlying data
     * This is internally used by the single-arg const_vector_single_value and to help with
     * serialization
     */
    template <int D = Eigen::Dynamic>
    const T& const_vector_single_value(
        int64_t index,
        int8_t vector_index,
        const std::vector<T>& data) const;
    /**
     * @brief Accesses the attribute using the specified scalar as the underlying data
     * This is internally used by the single-arg vector_single_value and to help with
     * serialization
     */
    template <int D = Eigen::Dynamic>
    T& vector_single_value(
        int64_t index,
        int8_t vector_index,
        std::vector<T>& data) const;

    // computes the "reserved size" but using the passed in data
    int64_t reserved_size(const std::vector<T>& data) const;

protected:
    std::vector<T> m_data;
    int64_t m_dimension = -1;
    T m_default_value = T(0);

public:
    std::string m_name;
};


} // namespace attribute
} // namespace wmtk
#if !defined(WMTK_ENABLED_DEV_MODE)
#include "Attribute.hxx"
#endif
