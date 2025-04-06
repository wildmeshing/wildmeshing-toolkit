#pragma once
#include <wmtk/PointMesh.hpp>

namespace wmtk::tests {
class DEBUG_PointMesh : public wmtk::PointMesh
{
public:
    using PointMesh::PointMesh;
    using PointMesh::operator=;
    using Mesh::get_flag_accessor;
    DEBUG_PointMesh(PointMesh&& m)
        : PointMesh(std::move(m))
    {}
    int64_t id(const wmtk::Tuple& tup) const
    {
        return PointMesh::id(tup, wmtk::PrimitiveType::Vertex);
    }
    using PointMesh::id;
    template <typename T>
    attribute::Attribute<T>& create_base_accessor(const attribute::TypedAttributeHandle<T>& handle)
    {
        return attribute::Accessor<T>(*this, handle).attribute();
    }

    template <typename T>
    const attribute::Attribute<T>& create_const_base_accessor(
        const attribute::TypedAttributeHandle<T>& handle) const
    {
        return attribute::Accessor<T>(*this, handle).attribute();
    }
    template <typename T>
    attribute::CachingAttribute<T>& create_index_accessor(
        const attribute::TypedAttributeHandle<T>& handle)
    {
        return attribute::Accessor<T>(*this, handle).attribute();
    }

    template <typename T>
    const attribute::CachingAttribute<T>& create_const_index_accessor(
        const attribute::TypedAttributeHandle<T>& handle) const
    {
        return attribute::Accessor<T>(*this, handle).attribute();
    }
};
} // namespace wmtk::tests
