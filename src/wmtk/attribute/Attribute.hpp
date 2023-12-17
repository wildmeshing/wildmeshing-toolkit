#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>

namespace wmtk {
class MeshWriter;

namespace attribute {
template <typename T>
class AccessorBase;

template <typename T>
class PerThreadAttributeScopeStacks;
template <typename T>
class AttributeScopeStack;

/**
 * This class stores data of type T in a vector.
 * If multiple values should be hold per index, the data will be automatically linearized.
 * For example, per index we have a 3-dimensional vector. Then the data vector will contain:
 * [x0,y0,z0,x1,y1,z1,...]
 */
template <typename T>
class Attribute
{
public:
    template <int R>
    using MapResultD = Eigen::Map<Eigen::Matrix<T, R, 1>>;
    template <int R>
    using ConstMapResultD = Eigen::Map<const Eigen::Matrix<T, R, 1>>;

    using MapResult = MapResultD<Eigen::Dynamic>;
    using ConstMapResult = ConstMapResultD<Eigen::Dynamic>;


    friend class AccessorBase<T>;
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
    Attribute(long dimension, T default_value = T(0), long size = 0);

    Attribute(const Attribute& o);
    Attribute(Attribute&& o);
    ~Attribute();
    Attribute& operator=(const Attribute& o);
    Attribute& operator=(Attribute&& o);
    ConstMapResult const_vector_attribute(const long index) const;
    MapResult vector_attribute(const long index);


    T const_scalar_attribute(const long index) const;
    T& scalar_attribute(const long index);

    /**
     * @brief Replace the internal data with `val`.
     */
    void set(std::vector<T> val);

    /**
     * @brief The total number of elements in a vector.
     * This is greater than the number of active values in the attribute, and the set of active
     * values is handled by a higher level abstraction
     */
    long reserved_size() const;

    /**
     * @brief The number of values for each index.
     */
    long dimension() const;
    void reserve(const long size);

    bool operator==(const Attribute<T>& o) const;

    void push_scope();
    void pop_scope(bool apply_updates);
    void clear_current_scope();

    // returns nullptr if no scope exists
    AttributeScopeStack<T>* get_local_scope_stack_ptr();

private:
    std::vector<T> m_data;
    std::unique_ptr<PerThreadAttributeScopeStacks<T>> m_scope_stacks;
    long m_dimension = -1;
    T m_default_value = T(0);
};
} // namespace attribute
} // namespace wmtk
