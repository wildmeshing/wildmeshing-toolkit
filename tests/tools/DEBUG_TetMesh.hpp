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
    DEBUG_TetMesh(const TetMesh& m);
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


    auto edge_tuple_between_v1_v2(const int64_t v1, const int64_t v2, const int64_t tid) const
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
    auto edge_tuple_between_v1_v2(
        const int64_t v1,
        const int64_t v2,
        const int64_t v3,
        const int64_t tid) const -> Tuple;
    auto edge_tuple_from_vids(const int64_t v1, const int64_t v2) const -> Tuple;
    auto face_tuple_from_vids(const int64_t v1, const int64_t v2, const int64_t v3) const -> Tuple;
    auto tet_tuple_from_vids(const int64_t v1, const int64_t v2, const int64_t v3, const int64_t v4)
        const -> Tuple;


    Tuple tuple_from_tet_id(const int64_t tid);

    template <typename T>
    attribute::AccessorBase<T> create_base_accessor(const TypedAttributeHandle<T>& handle)
    {
        return attribute::AccessorBase<T>(*this, handle);
    }

    template <typename T>
    attribute::AccessorBase<T> create_const_base_accessor(
        const TypedAttributeHandle<T>& handle) const
    {
        return attribute::AccessorBase<T>(const_cast<DEBUG_TetMesh&>(*this), handle);
    }
    template <typename T>
    attribute::AccessorBase<T> create_base_accessor(const TypedAttributeHandle<T>& handle) const
    {
        return create_const_base_accessor(handle);
    }

    const TypedAttributeHandle<int64_t>& t_handle(const PrimitiveType type) const;

    const TypedAttributeHandle<int64_t>& vt_handle() const;

    const TypedAttributeHandle<int64_t>& et_handle() const;

    const TypedAttributeHandle<int64_t>& ft_handle() const;

    void reserve_attributes(PrimitiveType type, int64_t size);

    int64_t id(const Tuple& tuple, PrimitiveType type) const override;
    int64_t id(const simplex::Simplex& s) const;

    using TetMesh::tuple_from_id;

    Accessor<int64_t> get_cell_hash_accessor();

    TetMeshOperationExecutor get_tmoe(const Tuple& t, Accessor<int64_t>& hash_accessor);

    int64_t valid_primitive_count(PrimitiveType type) const;

    Eigen::Matrix<int64_t, 4, 1> tv_from_tid(const int64_t tid) const
    {
        auto tv_accessor = create_base_accessor<int64_t>(t_handle(PrimitiveType::Vertex));
        return tv_accessor.vector_attribute(tid);
    }
};

} // namespace wmtk::tests_3d
