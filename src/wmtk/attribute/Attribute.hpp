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

    // if size < 0 then the internal data is not initialized
    Attribute(long dimension, long size);
    Attribute(long dimension);

    Attribute(const Attribute& o);
    Attribute(Attribute&& o);
    Attribute& operator=(const Attribute& o);
    Attribute& operator=(Attribute&& o);
    ConstMapResult const_vector_attribute(const long index) const;
    MapResult vector_attribute(const long index);


    T const_scalar_attribute(const long index) const;
    T& scalar_attribute(const long index);

    void set(std::vector<T> val);
    // The total number of elements in a vector. This is greater than the number of active values in
    // the attribute, and the set of active values is handled by a higher level abstraction
    long reserved_size() const;
    // The number of data for each element in the vector
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
};
} // namespace attribute
} // namespace wmtk
