#pragma once

#include <Eigen/Dense>
#include "AttributeHandle.hpp"

namespace wmtk {
class Mesh;

template <typename T>
class MeshAttributes;
template <typename T>
class Accessor
{
public:
    using MapResult = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>>;
    using ConstMapResult = Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>>;


    Accessor(MeshAttributes<T>& m, const AttributeHandle& handle);

    ConstMapResult vector_attribute(const long index) const;
    MapResult vector_attribute(const long index);

    T scalar_attribute(const long index) const;
    T& scalar_attribute(const long index);

private:
    MeshAttributes<T>& m_attribute;
    AttributeHandle m_handle;
};
} // namespace wmtk

#include "Accessor_impl.hpp"
