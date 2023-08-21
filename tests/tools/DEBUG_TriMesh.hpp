#pragma once
#include <wmtk/TriMesh.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>

namespace wmtk::tests {
class DEBUG_TriMesh : public TriMesh
{
public:
    using TriMesh::TriMesh;
    DEBUG_TriMesh(const TriMesh& m);
    DEBUG_TriMesh(TriMesh&& m);
    using TriMesh::operator=;


    bool operator==(const DEBUG_TriMesh& o) const;
    bool operator!=(const DEBUG_TriMesh& o) const;

    // uses spdlog to print out a variety of information about the mesh
    void print_state() const;


    auto edge_tuple_between_v1_v2(const long v1, const long v2, const long fid) const -> Tuple;

    Tuple tuple_from_face_id(const long fid) const;
    template <typename T>
    AccessorBase<T> create_base_accessor(const MeshAttributeHandle<T>& handle)
    {
        return AccessorBase<T>(*this, handle);
    }

    template <typename T>
    AccessorBase<T> create_const_base_accessor(const MeshAttributeHandle<T>& handle) const
    {
        return AccessorBase<T>(const_cast<DEBUG_TriMesh&>(*this), handle);
    }
    template <typename T>
    AccessorBase<T> create_base_accessor(const MeshAttributeHandle<T>& handle) const
    {
        return create_const_base_accessor(handle);
    }

    const MeshAttributeHandle<long>& f_handle(const PrimitiveType type) const;

    const MeshAttributeHandle<long>& vf_handle() const;

    const MeshAttributeHandle<long>& ef_handle() const;


    void reserve_attributes(PrimitiveType type, long size);


    long id(const Tuple& tuple, PrimitiveType type) const override;
    long id(const Simplex& s) const;
    /**
     * @brief returns the TriMeshOperationExecutor
     */
    using TriMesh::tuple_from_id;
    TriMeshOperationExecutor get_tmoe(const Tuple& t);
};

} // namespace wmtk::tests
