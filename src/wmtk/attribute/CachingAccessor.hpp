#pragma once

#include <optional>
#include "AccessorBase.hpp"
#include "AttributeAccessMode.hpp"

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


    CachingAccessor(
        const MeshAttributeHandle<T>& handle,
        AttributeAccessMode access_mode = AttributeAccessMode::Immediate);
    CachingAccessor(
        Mesh& m,
        const TypedAttributeHandle<T>& handle,
        AttributeAccessMode access_mode = AttributeAccessMode::Immediate);

    ~CachingAccessor();
    CachingAccessor(const CachingAccessor&) = delete;
    CachingAccessor& operator=(const CachingAccessor&) = delete;

    AttributeAccessMode access_mode() const;


    // returns the size of the underlying attribute

    //using BaseType::dimension; // const() -> long
    //using BaseType::reserved_size; // const() -> long

    //using BaseType::attribute; // access to Attribute object being used here
    //using BaseType::set_attribute; // (const vector<T>&) -> void
    // shows the depth of scope stacks if they exist, mostly for debug
    std::optional<long> stack_depth() const;

    bool has_stack() const;

    ConstMapResult const_vector_attribute(const long index) const;

    T const_scalar_attribute(const long index) const;

    MapResult vector_attribute(const long index);

    T& scalar_attribute(const long index);

    // deprecated because we should be more explicit in const/nonconst on internal interfaces
    ConstMapResult vector_attribute(const long index) const;
    //[[deprecated]] ConstMapResult vector_attribute(const long index) const;
    // deprecated because we should be more explicit in const/nonconst on internal interfaces
    T scalar_attribute(const long index) const;
    //[[deprecated]] T scalar_attribute(const long index) const;

    using BaseType::attribute;
    using BaseType::mesh;

    bool writing_enabled() const;

protected:
    BaseType& base_type() { return *this; }
    const BaseType& base_type() const { return *this; }

private:
    AttributeAccessMode m_mode;

    AttributeScopeStack<T>* m_cache_stack = nullptr;
};
} // namespace wmtk::attribute
