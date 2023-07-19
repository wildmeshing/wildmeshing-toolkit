#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>

namespace wmtk {
class MeshWriter;
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
    using MapResult = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>>;
    using ConstMapResult = Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>>;

    friend class AccessorBase<T>;
    void serialize(const std::string& name, const int dim, MeshWriter& writer) const;

    // if size < 0 then the internal data is not initialized
    Attribute(long stride, long size);
    ConstMapResult const_vector_attribute(const long index) const;
    MapResult vector_attribute(const long index);

    T const_scalar_attribute(const long index) const;
    T& scalar_attribute(const long index);

    void set(std::vector<T> val);
    long size() const;
    long stride() const;
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
    long m_stride = -1;
};
} // namespace wmtk
