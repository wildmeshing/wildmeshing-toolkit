#pragma once
#include <wmtk/PointMesh.hpp>

namespace wmtk::tests {
class DEBUG_PointMesh : public wmtk::PointMesh
{
public:
    using PointMesh::PointMesh;
    using PointMesh::operator=;
    DEBUG_PointMesh(const PointMesh& m)
        : PointMesh(m)
    {}
    DEBUG_PointMesh(PointMesh&& m)
        : PointMesh(std::move(m))
    {}
    int64_t id(const wmtk::Tuple& tup) const
    {
        return PointMesh::id(tup, wmtk::PrimitiveType::Vertex);
    }
    template <typename T>
    attribute::AccessorBase<T> create_base_accessor(const MeshAttributeHandle<T>& handle)
    {
        return attribute::AccessorBase<T>(*this, handle);
    }

    template <typename T>
    attribute::AccessorBase<T> create_const_base_accessor(
        const MeshAttributeHandle<T>& handle) const
    {
        return attribute::AccessorBase<T>(const_cast<DEBUG_PointMesh&>(*this), handle);
    }
    template <typename T>
    attribute::CachingAccessor<T> create_index_accessor(const MeshAttributeHandle<T>& handle)
    {
        return attribute::CachingAccessor<T>(*this, handle);
    }

    template <typename T>
    attribute::CachingAccessor<T> create_const_index_accessor(
        const MeshAttributeHandle<T>& handle) const
    {
        return attribute::CachingAccessor<T>(const_cast<DEBUG_PointMesh&>(*this), handle);
    }
};
} // namespace wmtk::tests
