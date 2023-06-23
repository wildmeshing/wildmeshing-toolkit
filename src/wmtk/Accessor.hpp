#pragma once

#include <Eigen/Dense>
#include "AttributeHandle.hpp"
#include "Tuple.h"

namespace wmtk {
class Mesh;

template <typename T>
class MeshAttributes;
template <typename T>
class Accessor
{
public:
    friend class Mesh;
    using MapResult = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>>;
    using ConstMapResult = Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>>;


    Accessor(Mesh& m, const MeshAttributeHandle<T>& handle);

    ConstMapResult vector_attribute(const Tuple& t) const;
    MapResult vector_attribute(const Tuple& t);

    T scalar_attribute(const Tuple& t) const;
    T& scalar_attribute(const Tuple& t);

private:
    ConstMapResult vector_attribute(const long index) const;
    MapResult vector_attribute(const long index);

    T scalar_attribute(const long index) const;
    T& scalar_attribute(const long index);

private:
    MeshAttributes<T>& attributes();
    const MeshAttributes<T>& attributes() const;

    Mesh& m_mesh;
    MeshAttributeHandle<T> m_handle;
};

template <typename T>
Accessor(Mesh& mesh, const MeshAttributeHandle<T>&) -> Accessor<T>;
} // namespace wmtk

