#pragma once
#include <wmtk/TriMesh.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include "DEBUG_MultiMeshManager.hpp"

namespace wmtk::tests {
class DEBUG_TriMesh : public TriMesh //, public virtual DEBUG_Mesh
{
public:
    using TriMesh::get_flag_accessor;
    using TriMesh::TriMesh;
    DEBUG_TriMesh(const TriMesh& m);
    DEBUG_TriMesh(TriMesh&& m);
    using TriMesh::operator=;


    bool operator==(const DEBUG_TriMesh& o) const;
    bool operator!=(const DEBUG_TriMesh& o) const;

    // uses spdlog to print out a variety of information about the mesh
    void print_state() const;
    DEBUG_MultiMeshManager& multi_mesh_manager()
    {
        return reinterpret_cast<DEBUG_MultiMeshManager&>(m_multi_mesh_manager);
    }
    const DEBUG_MultiMeshManager& multi_mesh_manager() const
    {
        return reinterpret_cast<const DEBUG_MultiMeshManager&>(m_multi_mesh_manager);
    }

    using TriMesh::m_attribute_manager;

    void print_vf() const;
    Eigen::Matrix<int64_t, 3, 1> fv_from_fid(const int64_t fid) const;

    auto edge_tuple_between_v1_v2(const int64_t v1, const int64_t v2, const int64_t fid) const
        -> Tuple;

    auto edge_tuple_from_vids(const int64_t v1, const int64_t v2) const -> Tuple;
    auto face_tuple_from_vids(const int64_t v1, const int64_t v2, const int64_t v3) const -> Tuple;

    Tuple tuple_from_face_id(const int64_t fid) const;
    template <typename T>
    attribute::AccessorBase<T> create_base_accessor(const TypedAttributeHandle<T>& handle)
    {
        return attribute::AccessorBase<T>(*this, handle);
    }

    template <typename T>
    attribute::AccessorBase<T> create_const_base_accessor(
        const TypedAttributeHandle<T>& handle) const
    {
        return attribute::AccessorBase<T>(const_cast<DEBUG_TriMesh&>(*this), handle);
    }
    template <typename T>
    attribute::AccessorBase<T> create_base_accessor(const TypedAttributeHandle<T>& handle) const
    {
        return create_const_base_accessor(handle);
    }

    const TypedAttributeHandle<int64_t>& f_handle(const PrimitiveType type) const;

    const TypedAttributeHandle<int64_t>& vf_handle() const;

    const TypedAttributeHandle<int64_t>& ef_handle() const;


    void reserve_attributes(PrimitiveType type, int64_t size);
    void reserve_more_attributes(const std::vector<int64_t>& sizes);


    int64_t id(const Tuple& tuple, PrimitiveType type) const override;
    int64_t id(const simplex::Simplex& s) const;
    /**
     * @brief returns the TriMeshOperationExecutor
     */
    using TriMesh::tuple_from_id;

    using TriMesh::custom_attributes;

    Accessor<int64_t> get_cell_hash_accessor();

    TriMeshOperationExecutor get_tmoe(const Tuple& t, Accessor<int64_t>& hash_accessor);
};

} // namespace wmtk::tests
