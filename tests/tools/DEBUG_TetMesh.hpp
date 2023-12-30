#pragma once
#include <wmtk/TetMesh.hpp>
#include <wmtk/TetMeshOperationExecutor.hpp>

namespace wmtk::tests_3d {
class DEBUG_TetMesh : public TetMesh
{
public:
    using TetMesh::TetMesh;
    DEBUG_TetMesh(const TetMesh& m);
    DEBUG_TetMesh(TetMesh&& m);
    using TetMesh::operator=;

    bool operator==(const DEBUG_TetMesh& o) const;
    bool operator!=(const DEBUG_TetMesh& o) const;

    // uses spdlog to print out a variety of information about the mesh
    void print_state() const;


    auto edge_tuple_between_v1_v2(const long v1, const long v2, const long tid) const -> Tuple;
    /**
     * @brief return a tuple with edge v1v2 and face v1v2v3 in tet tid
     *
     * @param v1
     * @param v2
     * @param v3
     * @param tid
     * @return Tuple
     */
    auto edge_tuple_between_v1_v2(const long v1, const long v2, const long v3, const long tid) const
        -> Tuple;
    auto edge_tuple_from_vids(const long v1, const long v2) const -> Tuple;
    auto face_tuple_from_vids(const long v1, const long v2, const long v3) const -> Tuple;
    auto tet_tuple_from_vids(const long v1, const long v2, const long v3, const long v4) const
        -> Tuple;


    Tuple tuple_from_tet_id(const long tid);

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

    const TypedAttributeHandle<long>& t_handle(const PrimitiveType type) const;

    const TypedAttributeHandle<long>& vt_handle() const;

    const TypedAttributeHandle<long>& et_handle() const;

    const TypedAttributeHandle<long>& ft_handle() const;

    void reserve_attributes(PrimitiveType type, long size);

    long id(const Tuple& tuple, PrimitiveType type) const override;
    long id(const simplex::Simplex& s) const;

    using TetMesh::tuple_from_id;

    Accessor<long> get_cell_hash_accessor();

    TetMeshOperationExecutor get_tmoe(const Tuple& t, Accessor<long>& hash_accessor);

    long valid_primitive_count(PrimitiveType type) const;
};

} // namespace wmtk::tests_3d
