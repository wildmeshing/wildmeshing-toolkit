#pragma once

#include <memory>
#include <type_traits>
#include "AttributeHandle.hpp"
#include "Tuple.hpp"

#include <Eigen/Dense>

namespace wmtk {

template <typename T>
class MeshAttributes;


// The basic implementation of an accessor using indices.
// This should never be
template <typename T, bool IsConst = false>
class AccessorBase
{
public:
    using MeshType = std::conditional_t<IsConst, const Mesh, Mesh>;
    using MeshAttributesType =
        std::conditional_t<IsConst, const MeshAttributes<T>, MeshAttributes<T>>;

    using MapResult = typename Eigen::Matrix<T, Eigen::Dynamic, 1>::MapType;
    using ConstMapResult = typename Eigen::Matrix<T, Eigen::Dynamic, 1>::ConstMapType;

    using MapResultType = std::conditional_t<IsConst, ConstMapResult, MapResult>;
    using TT = std::conditional_t<IsConst, T, T&>;


public:
    // returns the size of the underlying attribute
    long size() const;
    long stride() const;

protected:
    AccessorBase(MeshType& m, const MeshAttributeHandle<T>& handle);
    ~AccessorBase();

    void set_attribute(const std::vector<T>& value);


    ConstMapResult vector_attribute(const long index) const;
    MapResultType vector_attribute(const long index);

    T scalar_attribute(const long index) const;
    TT scalar_attribute(const long index);

private:
    MeshAttributesType& attributes();
    const MeshAttributesType& attributes() const;

    MeshType& m_mesh;
    MeshAttributeHandle<T> m_handle;
};


} // namespace wmtk
