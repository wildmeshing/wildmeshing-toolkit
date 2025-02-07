#pragma once

#include <optional>
#include "AccessorBase.hpp"

namespace wmtk {
class Mesh;
class TetMesh;
class TriMesh;
} // namespace wmtk
namespace wmtk::attribute {

namespace internal {
template <typename T>
class AttributeTransactionStack;
}

/**
 * An accessor for cached attribute values. This accessor or any of its derivatives should be used
 * for accessing attributes.
 */
template <typename T, int Dim = Eigen::Dynamic>
class CachingAccessor : public AccessorBase<T, Dim>
{
public:
    friend class wmtk::Mesh;
    friend class wmtk::TetMesh;
    friend class wmtk::TriMesh;
    using Scalar = T;

    using BaseType = AccessorBase<T, Dim>;

    template <int D = Dim>
    using ConstMapResult =
        typename BaseType::template ConstMapResult<D>; // Eigen::Map<const VectorX<T>>
    template <int D = Dim>
    using MapResult = typename BaseType::template MapResult<D>; // Eigen::Map<VectorX<T>>


    CachingAccessor(Mesh& m, const TypedAttributeHandle<T>& handle);
    CachingAccessor(const Mesh& m, const TypedAttributeHandle<T>& handle);

    ~CachingAccessor();
    CachingAccessor(const CachingAccessor&) = delete;
    CachingAccessor& operator=(const CachingAccessor&) = delete;
    CachingAccessor(CachingAccessor&&) = default;
    CachingAccessor& operator=(CachingAccessor&&) = default;


    // returns the size of the underlying attribute

    //using BaseType::dimension; // const() -> int64_t
    //using BaseType::reserved_size; // const() -> int64_t

    //using BaseType::attribute; // access to Attribute object being used here
    //using BaseType::set_attribute; // (const vector<T>&) -> void
    // shows the depth of scope stacks if they exist, mostly for debug
    int64_t stack_depth() const;

    bool has_stack() const;

    template <int D = Dim>
    ConstMapResult<std::max(D, Dim)> const_vector_attribute(const int64_t index) const;

    T const_scalar_attribute(const int64_t index) const;
    T const_vector_single_value(const int64_t index, const int8_t vector_index) const;

    template <int D = Dim>
    MapResult<std::max(D, Dim)> vector_attribute(const int64_t index);

    T& scalar_attribute(const int64_t index);
    T& vector_single_value(const int64_t index, const int8_t vector_index);

    // deprecated because we should be more explicit in const/nonconst on internal interfaces
    ConstMapResult<> vector_attribute(const int64_t index) const;
    // deprecated because we should be more explicit in const/nonconst on internal interfaces
    T scalar_attribute(const int64_t index) const;

    using BaseType::attribute;
    using BaseType::mesh;

    bool writing_enabled() const;

protected:
    BaseType& base_type() { return *this; }
    const BaseType& base_type() const { return *this; }

    CachingAttribute<T>& get_cache_stack();
    const CachingAttribute<T>& get_cache_stack() const;
};
} // namespace wmtk::attribute
#if !defined(WMTK_ENABLED_DEV_MODE)
#include "CachingAccessor.hxx"
#endif
