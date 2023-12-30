#pragma once

// #include <memory>
// #include <optional>
// #include <type_traits>
// #include "Tuple.hpp"
//#include "attribute/AccessorBase.hpp"
//#include "attribute/AttributeAccessMode.hpp"
//#include "attribute/AttributeHandle.hpp"
#include "attribute/ConstAccessor.hpp"
#include "attribute/MutableAccessor.hpp"

#include <Eigen/Dense>

namespace wmtk {
// class Mesh;
// class TriMesh;
// class TetMesh;
// class PointMesh;
// namespace attribute {
// template <typename T>
// class AttributeScopeStack;
//
// template <typename T>
// class AttributeCache;
//} // namespace attribute

template <typename T, bool IsConst = false>
using Accessor =
    std::conditional_t<IsConst, attribute::ConstAccessor<T>, attribute::MutableAccessor<T>>;


/*
template <typename T, bool IsConst = false>
class Accessor : public attribute::AccessorBase<T>
{
public:
    friend class Mesh;
    friend class PointMesh;
    friend class TriMesh;
    friend class TetMesh;
    using Scalar = T;

    friend class attribute::AttributeCache<T>;
    using BaseType = attribute::AccessorBase<T>;
    using MeshType = std::conditional_t<IsConst, const Mesh, Mesh>; // const correct Mesh object

    using MapResult = typename BaseType::MapResult; // Eigen::Map<VectorX<T>>
    using ConstMapResult = typename BaseType::ConstMapResult; // Eigen::Map<const VectorX<T>>


    using MapResultT =
        std::conditional_t<IsConst, ConstMapResult, MapResult>; // MapResult or ConstMapResult for
                                                                // constness


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

    using BaseType::dimension; // const() -> int64_t
    using BaseType::reserved_size; // const() -> int64_t

    using BaseType::attribute; // access to Attribute object being used here
    using BaseType::set_attribute; // (const vector<T>&) -> void
    // shows the depth of scope stacks if they exist, mostly for debug
    std::optional<int64_t> stack_depth() const;

protected:
    ConstMapResult cacheable_const_vector_attribute(const int64_t index) const;
    MapResultT cacheable_vector_attribute(const int64_t index);

    T cacheable_const_scalar_attribute(const int64_t index) const;
    TT cacheable_scalar_attribute(const int64_t index);

    ConstMapResult const_vector_attribute(const int64_t index) const;
    ConstMapResult vector_attribute(const int64_t index) const;
    MapResultT vector_attribute(const int64_t index);

    T const_scalar_attribute(const int64_t index) const;
    T scalar_attribute(const int64_t index) const;
    TT scalar_attribute(const int64_t index);

    // using BaseType::scalar_attribute;
    // using BaseType::vector_attribute;
    int64_t index(const Tuple& t) const;


private:
    MeshType& m_mesh;
    AttributeAccessMode m_mode;

    attribute::AttributeScopeStack<T>* m_cache_stack = nullptr;
};

// template <typename T>
// Accessor(Mesh& mesh, const MeshAttributeHandle<T>&) -> Accessor<T>;
// template <typename T>
// Accessor(const Mesh& mesh, const MeshAttributeHandle<T>&) -> Accessor<const T>;


*/
template <typename T>
using ConstAccessor = Accessor<T, true>;
} // namespace wmtk
