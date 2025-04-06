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
    DEBUG_TriMesh(TriMesh&& m);

    using TriMesh::id;
    using TriMesh::id_vertex;
    using TriMesh::m_fe_accessor;
    using TriMesh::m_ff_accessor;
    using TriMesh::m_fv_accessor;
    using TriMesh::tuple_from_global_ids;
    using TriMesh::tuple_from_id;


    DEBUG_TriMesh& operator=(TriMesh&& o);

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

    auto edge_tuple_with_vs_and_t(const int64_t v1, const int64_t v2, const int64_t fid) const
        -> Tuple;

    using TriMesh::vertex_tuple_from_id;
    auto edge_tuple_from_vids(const int64_t v1, const int64_t v2) const -> Tuple;
    auto face_tuple_from_vids(const int64_t v1, const int64_t v2, const int64_t v3) const -> Tuple;

    Tuple tuple_from_face_id(const int64_t fid) const;
    template <typename T>
    attribute::Attribute<T>& create_base_accessor(const TypedAttributeHandle<T>& handle)
    {
        return attribute::Accessor<T>(*this, handle).attribute();
    }

    template <typename T>
    attribute::Attribute<T>& create_const_base_accessor(const TypedAttributeHandle<T>& handle) const
    {
        return attribute::Accessor<T>(*this, handle).attribute();
    }
    template <typename T>
    attribute::Attribute<T>& create_base_accessor(const TypedAttributeHandle<T>& handle) const
    {
        return create_const_base_accessor(handle);
    }

    const TypedAttributeHandle<int64_t>& f_handle(const PrimitiveType type) const;

    const TypedAttributeHandle<int64_t>& vf_handle() const;

    const TypedAttributeHandle<int64_t>& ef_handle() const;


    void reserve_attributes(PrimitiveType type, int64_t size);
    void reserve_more_attributes(const std::vector<int64_t>& sizes);


    /**
     * @brief returns the TriMeshOperationExecutor
     */

    using TriMesh::custom_attributes;

    TriMeshOperationExecutor get_tmoe(const Tuple& t);
};

} // namespace wmtk::tests
