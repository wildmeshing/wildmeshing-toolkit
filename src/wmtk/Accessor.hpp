#pragma once

#include <memory>
#include <type_traits>
#include "AttributeHandle.hpp"
#include "Tuple.hpp"

#include <Eigen/Dense>

namespace wmtk {
class Mesh;
class TriMesh;
class TetMesh;
enum class AccessorCacheMode { Immediate, ReadBuffered, WriteBuffered, ReadWriteBuffered };

template <typename T>
class MeshAttributes;

template <typename T>
class AccessorCache;

template <typename T, bool IsConst = false>
class Accessor
{
public:
    friend class Mesh;
    friend class TriMesh;
    friend class TetMesh;
    using MeshType = std::conditional_t<IsConst, const Mesh, Mesh>;
    using MeshAttributesType =
        std::conditional_t<IsConst, const MeshAttributes<T>, MeshAttributes<T>>;

    using MapResult = typename Eigen::Matrix<T, Eigen::Dynamic, 1>::MapType;
    using ConstMapResult = typename Eigen::Matrix<T, Eigen::Dynamic, 1>::ConstMapType;

    using MapResultT = std::conditional_t<IsConst, ConstMapResult, MapResult>;
    using TT = std::conditional_t<IsConst, T, T&>;


    Accessor(
        MeshType& m,
        const MeshAttributeHandle<T>& handle,
        AccessorCacheMode mode = AccessorCacheMode::Immediate);
    ~Accessor();

    Accessor(const Accessor&) = delete;
    Accessor& operator=(const Accessor&) = delete;

    ConstMapResult vector_attribute(const Tuple& t) const;
    MapResultT vector_attribute(const Tuple& t);

    T scalar_attribute(const Tuple& t) const;
    TT scalar_attribute(const Tuple& t);

    void set_attribute(const std::vector<T>& value);

    //returns the size of the underlying attribute
    long size() const;

private:
    ConstMapResult vector_attribute(const long index) const;
    MapResultT vector_attribute(const long index);

    T scalar_attribute(const long index) const;
    TT scalar_attribute(const long index);

private:
    MeshAttributesType& attributes();
    const MeshAttributesType& attributes() const;

    MeshType& m_mesh;
    MeshAttributeHandle<T> m_handle;

    AccessorCacheMode m_cache_mode = AccessorCacheMode::Immediate;
    std::unique_ptr<AccessorCache<T>> m_cache;
};

// template <typename T>
// Accessor(Mesh& mesh, const MeshAttributeHandle<T>&) -> Accessor<T>;
// template <typename T>
// Accessor(const Mesh& mesh, const MeshAttributeHandle<T>&) -> Accessor<const T>;


template <typename T>
using ConstAccessor = Accessor<T, true>;
} // namespace wmtk
