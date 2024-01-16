#pragma once

#include <optional>
#include "AccessorBase.hpp"

namespace wmtk {
class Mesh;
class TetMesh;
class TriMesh;
} // namespace wmtk
namespace wmtk::attribute {

template <typename T>
class AttributeCache;

/**
 * An accessor for cached attribute values. This accessor or any of its derivatives should be used
 * for accessing attributes.
 */
template <typename T>
class CachingAccessor : public AccessorBase<T>
{
public:
    friend class wmtk::Mesh;
    friend class wmtk::TetMesh;
    friend class wmtk::TriMesh;
    using Scalar = T;

    friend class AttributeCache<T>;
    using BaseType = AccessorBase<T>;

    using ConstMapResult = typename BaseType::ConstMapResult; // Eigen::Map<const VectorX<T>>
    using MapResult = typename BaseType::MapResult; // Eigen::Map<VectorX<T>>


    CachingAccessor(Mesh& m, const TypedAttributeHandle<T>& handle);

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

    ConstMapResult const_vector_attribute(const int64_t index) const;

    T const_scalar_attribute(const int64_t index) const;

    MapResult vector_attribute(const int64_t index);

    T& scalar_attribute(const int64_t index);

    // deprecated because we should be more explicit in const/nonconst on internal interfaces
    ConstMapResult vector_attribute(const int64_t index) const;
    // deprecated because we should be more explicit in const/nonconst on internal interfaces
    T scalar_attribute(const int64_t index) const;

    using BaseType::attribute;
    using BaseType::mesh;

    bool writing_enabled() const;

protected:
    BaseType& base_type() { return *this; }
    const BaseType& base_type() const { return *this; }

private:
    AttributeScopeStack<T>& m_cache_stack;
};
} // namespace wmtk::attribute
#include "CachingAccessor.hxx"
