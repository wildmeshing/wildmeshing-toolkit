#pragma once

#include <memory>
#include <type_traits>
#include "attribute/AttributeAccessMode.hpp"
#include "attribute/AccessorBase.hpp"
#include "AttributeHandle.hpp"
#include "Tuple.hpp"

#include <Eigen/Dense>

namespace wmtk {
class Mesh;
class TriMesh;
class TetMesh;

template <typename T>
class AccessorCache;

template <typename T, bool IsConst = false>
class Accessor : public AccessorBase<T>
{
public:
    friend class Mesh;
    friend class TriMesh;
    friend class TetMesh;
    using Scalar = T;

    friend class AccessorCache<T>;
    using BaseType = AccessorBase<T>;
    using MeshType = std::conditional_t<IsConst, const Mesh, Mesh>; // const correct Mesh object

    using MapResult = typename BaseType::MapResult; // Eigen::Map<VectorX<T>>
    using ConstMapResult = typename BaseType::ConstMapResult; // Eigen::Map<const VectorX<T>>


    using MapResultT = std::conditional_t<IsConst, ConstMapResult, MapResult>; // MapResult or ConstMapResult for constness


    // T or T& for const correctness
    using TT = std::conditional_t<IsConst, T, T&>;


    Accessor(
        MeshType& m,
        const MeshAttributeHandle<T>& handle,
        AttributeAccessMode access_mode = AttributeAccessMode::Immediate);

    ~Accessor();
    Accessor(const Accessor&) = delete;
    Accessor& operator=(const Accessor&) = delete;

    AttributeAccessMode access_mode() const;

    ConstMapResult const_vector_attribute(const Tuple& t) const;

    ConstMapResult vector_attribute(const Tuple& t) const;
    MapResultT vector_attribute(const Tuple& t);

    T const_scalar_attribute(const Tuple& t) const;
    T scalar_attribute(const Tuple& t) const;
    TT scalar_attribute(const Tuple& t);

    // returns the size of the underlying attribute

    using BaseType::size; // const() -> long
    using BaseType::stride; // const() -> long

    using BaseType::set_attribute; // (const vector<T>&) -> void
protected:
    ConstMapResult cacheable_const_vector_attribute(const long index) const;
    MapResultT cacheable_vector_attribute(const long index);

    T cacheable_const_scalar_attribute(const long index) const;
    TT cacheable_scalar_attribute(const long index);
    using BaseType::scalar_attribute;
    using BaseType::vector_attribute;

private:
    AttributeAccessMode m_mode;

    std::shared_ptr<AccessorCache<T>> m_cache;
};

// template <typename T>
// Accessor(Mesh& mesh, const MeshAttributeHandle<T>&) -> Accessor<T>;
// template <typename T>
// Accessor(const Mesh& mesh, const MeshAttributeHandle<T>&) -> Accessor<const T>;


template <typename T>
using ConstAccessor = Accessor<T, true>;
} // namespace wmtk
