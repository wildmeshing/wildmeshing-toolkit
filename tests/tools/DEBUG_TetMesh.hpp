#pragma once
#include <wmtk/TetMesh.hpp>
#include <wmtk/TetMeshOperationExecutor.hpp>
#include "DEBUG_MultiMeshManager.hpp"
namespace wmtk::tests_3d {
class DEBUG_TetMesh : public TetMesh
{
public:
    using TetMesh::get_flag_accessor;
    using TetMesh::TetMesh;
    DEBUG_TetMesh(TetMesh&& m);
    using TetMesh::operator=;

    bool operator==(const DEBUG_TetMesh& o) const;
    bool operator!=(const DEBUG_TetMesh& o) const;

    // uses spdlog to print out a variety of information about the mesh
    void print_state() const;

    wmtk::tests::DEBUG_MultiMeshManager& multi_mesh_manager()
    {
        return reinterpret_cast<wmtk::tests::DEBUG_MultiMeshManager&>(m_multi_mesh_manager);
    }

    using TetMesh::m_attribute_manager;


    auto edge_tuple_with_vs_and_t(const int64_t v1, const int64_t v2, const int64_t tid) const
        -> Tuple;
    /**
     * @brief return a tuple with edge v1v2 and face v1v2v3 in tet tid
     *
     * @param v1
     * @param v2
     * @param v3
     * @param tid
     * @return Tuple
     */
    auto face_tuple_with_vs_and_t(
        const int64_t v1,
        const int64_t v2,
        const int64_t v3,
        const int64_t tid) const -> Tuple;
    using TetMesh::vertex_tuple_from_id;
    auto edge_tuple_from_vids(const int64_t v1, const int64_t v2) const -> Tuple;
    auto face_tuple_from_vids(const int64_t v1, const int64_t v2, const int64_t v3) const -> Tuple;
    auto tet_tuple_from_vids(const int64_t v1, const int64_t v2, const int64_t v3, const int64_t v4)
        const -> Tuple;


    Tuple tuple_from_tet_id(const int64_t tid);

    template <typename T>
    attribute::Attribute<T>& create_base_accessor(const TypedAttributeHandle<T>& handle)
    {
        return attribute::Accessor<T>(*this, handle).index_access();
    }

    template <typename T>
    const attribute::Attribute<T>& create_const_base_accessor(
        const TypedAttributeHandle<T>& handle) const
    {
        const attribute::Accessor<T> acc(const_cast<DEBUG_TetMesh&>(*this), handle);
        return acc.attribute();
    }
    template <typename T>
    const attribute::Attribute<T>& create_base_accessor(const TypedAttributeHandle<T>& handle) const
    {
        return create_const_base_accessor(handle);
    }

    const TypedAttributeHandle<int64_t>& t_handle(const PrimitiveType type) const;

    const TypedAttributeHandle<int64_t>& vt_handle() const;

    const TypedAttributeHandle<int64_t>& et_handle() const;

    const TypedAttributeHandle<int64_t>& ft_handle() const;

    void reserve_attributes(PrimitiveType type, int64_t size);

    using TetMesh::id;

    using TetMesh::tuple_from_id;

    TetMeshOperationExecutor get_tmoe(const Tuple& t);

    int64_t valid_primitive_count(PrimitiveType type) const;

    Eigen::Matrix<int64_t, 4, 1> tv_from_tid(const int64_t tid) const
    {
        auto& tv_accessor = create_base_accessor<int64_t>(t_handle(PrimitiveType::Vertex));
        return tv_accessor.const_vector_attribute(tid);
    }
};

} // namespace wmtk::tests_3d
