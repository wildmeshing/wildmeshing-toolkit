#pragma once
#include "MeshAttributes.hpp"


namespace wmtk {
template <typename T>
class Accessor
{
public:
    using MapResult = Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>>;
    using ConstMapResult = Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>>;


    Accessor(Mesh& m, const AttributeHandle& handle);

    ConstMapResult<T> vector_attribute(const long index) const;
    MapResult<T> vector_attribute(const long index);

    T scalar_attribute(const long index) const;
    T& scalar_attribute(const long index);
};

private:
MeshAttribute<T>& m_attribute;
AttributeHandle m_handle;
}; // namespace wmtk

#include "Accessor_impl.hpp"
