#include "DEBUG_TriMesh.hpp"
#include <catch2/catch_test_macros.hpp>

namespace wmtk::tests {

DEBUG_TriMesh::DEBUG_TriMesh(const TriMesh& m)
    : TriMesh(m)
{}
DEBUG_TriMesh::DEBUG_TriMesh(TriMesh&& m)
    : TriMesh(std::move(m))
{}


bool DEBUG_TriMesh::operator==(const DEBUG_TriMesh& o) const
{
    return static_cast<const TriMesh&>(*this) == static_cast<const TriMesh&>(o);
}
bool DEBUG_TriMesh::operator!=(const DEBUG_TriMesh& o) const
{
    return !(*this == o);
}


void DEBUG_TriMesh::print_state() const {}


auto DEBUG_TriMesh::edge_tuple_between_v1_v2(const long v1, const long v2, const long fid) const
    -> Tuple
{
    ConstAccessor<long> fv = create_accessor<long>(m_fv_handle);
    auto fv_base = create_base_accessor<long>(m_fv_handle);
    Tuple face = face_tuple_from_id(fid);
    auto fv0 = fv.vector_attribute(face);
    REQUIRE(fv0 == fv_base.vector_attribute(fid));
    long local_vid1 = -1, local_vid2 = -1;
    for (long i = 0; i < fv0.size(); ++i) {
        if (fv0[i] == v1) {
            local_vid1 = i;
        }
        if (fv0[i] == v2) {
            local_vid2 = i;
        }
    }
    return Tuple(local_vid1, (3 - local_vid1 - local_vid2) % 3, -1, fid, 0);
}

Tuple DEBUG_TriMesh::tuple_from_face_id(const long fid) const
{
    return tuple_from_id(PrimitiveType::Face, fid);
}


const MeshAttributeHandle<long>& DEBUG_TriMesh::f_handle(const PrimitiveType type) const
{
    switch (type) {
    case PrimitiveType::Vertex: return m_fv_handle;
    case PrimitiveType::Edge: return m_fe_handle;
    case PrimitiveType::Face: return m_ff_handle;
    default: throw std::runtime_error("Invalid PrimitiveType");
    }
}

const MeshAttributeHandle<long>& DEBUG_TriMesh::vf_handle() const
{
    return m_vf_handle;
}

const MeshAttributeHandle<long>& DEBUG_TriMesh::ef_handle() const
{
    return m_ef_handle;
}


void DEBUG_TriMesh::reserve_attributes(PrimitiveType type, long size)
{
    Mesh::reserve_attributes(type, size);
}


long DEBUG_TriMesh::id(const Tuple& tuple, PrimitiveType type) const
{
    return TriMesh::id(tuple, type);
}
long DEBUG_TriMesh::id(const Simplex& s) const
{
    return id(s.tuple(), s.primitive_type());
}
/**
 * @brief returns the TriMeshOperationExecutor
 */
auto DEBUG_TriMesh::get_tmoe(const Tuple& t) -> TriMeshOperationExecutor
{
    return TriMeshOperationExecutor(*this, t);
}
} // namespace wmtk::tests
