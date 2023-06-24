#pragma once

#include <type_traits>
#include "AttributeHandle.hpp"
#include "Tuple.hpp"

#include <Eigen/Dense>

namespace wmtk {
class Mesh;
class TriMesh;

template <typename T>
class MeshAttributes;
template <typename T>
class Accessor
{
public:
    friend class Mesh;
    friend class TriMesh;
    using Type = std::remove_const_t<T>;

    using MapResult = typename Eigen::Matrix<Type, Eigen::Dynamic, 1>::MapType;
    using ConstMapResult = typename Eigen::Matrix<Type, Eigen::Dynamic, 1>::ConstMapType;

    constexpr static bool IsConst = std::is_const_v<T>;
    using MeshType = std::conditional_t<IsConst, const Mesh, Mesh>;


    Accessor(MeshType& m, const MeshAttributeHandle<Type>& handle);

    ConstMapResult vector_attribute(const Tuple& t) const;
    MapResult vector_attribute(const Tuple& t);

    T scalar_attribute(const Tuple& t) const;
    T& scalar_attribute(const Tuple& t);

    ConstMapResult vector_attribute(const long index) const;
    MapResult vector_attribute(const long index);

    T scalar_attribute(const long index) const;
    T& scalar_attribute(const long index);

private:
    MeshAttributes<Type>& attributes();
    const MeshAttributes<Type>& attributes() const;

    MeshType& m_mesh;
    MeshAttributeHandle<Type> m_handle;
};

// template <typename T>
// Accessor(Mesh& mesh, const MeshAttributeHandle<T>&) -> Accessor<T>;
// template <typename T>
// Accessor(const Mesh& mesh, const MeshAttributeHandle<T>&) -> Accessor<const T>;


template <typename T>
using ConstAccessor = Accessor<const T>;
} // namespace wmtk
