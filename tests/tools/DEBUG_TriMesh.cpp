#include "DEBUG_TriMesh.hpp"
#include <catch2/catch_test_macros.hpp>

namespace wmtk::tests {
// This is to allow us to do hacky dynamic casts on the DEBUG TriMesh
constexpr std::conditional_t<sizeof(DEBUG_TriMesh) == sizeof(TriMesh), int, void>
    DONT_ALLOW_DEBUG_TO_ADD_MEMBERS = 1;

DEBUG_TriMesh& DEBUG_TriMesh::operator=(TriMesh&& o)
{
    TriMesh::operator=(std::move(o));
    return *this;
}
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

void DEBUG_TriMesh::print_vf() const
{
    auto& fv_accessor = create_base_accessor<int64_t>(f_handle(PrimitiveType::Vertex));
    auto f_flag_accessor = get_flag_accessor(PrimitiveType::Triangle);
    for (int64_t id = 0; id < capacity(PrimitiveType::Triangle); ++id) {
        auto fv = fv_accessor.const_vector_attribute(id);
        if (!f_flag_accessor.is_active(tuple_from_id(PrimitiveType::Triangle, id))) {
            std::cout << "face " << id << " is deleted" << std::endl;
        } else {
            std::cout << fv(0) << " " << fv(1) << " " << fv(2) << std::endl;
        }
    }
}

Eigen::Matrix<int64_t, 3, 1> DEBUG_TriMesh::fv_from_fid(const int64_t fid) const
{
    auto& fv_accessor = create_base_accessor<int64_t>(f_handle(PrimitiveType::Vertex));
    return fv_accessor.vector_attribute(fid);
}
void DEBUG_TriMesh::reserve_more_attributes(const std::vector<int64_t>& sizes)
{
    Mesh::reserve_more_attributes(sizes);
}

auto DEBUG_TriMesh::edge_tuple_with_vs_and_t(const int64_t v1, const int64_t v2, const int64_t fid)
    const -> Tuple
{
    const attribute::Accessor<int64_t> fv = create_const_accessor<int64_t>(m_fv_handle);
    auto& fv_base = create_base_accessor<int64_t>(m_fv_handle);
    Tuple face = face_tuple_from_id(fid);
    auto fv0 = fv.const_vector_attribute(face);
    REQUIRE(fv0 == fv_base.const_vector_attribute(fid));
    int64_t local_vid1 = -1, local_vid2 = -1;
    for (int64_t i = 0; i < fv0.size(); ++i) {
        if (fv0[i] == v1) {
            local_vid1 = i;
        }
        if (fv0[i] == v2) {
            local_vid2 = i;
        }
    }
    return Tuple(local_vid1, (3 - local_vid1 - local_vid2) % 3, -1, fid);
}

auto DEBUG_TriMesh::edge_tuple_from_vids(const int64_t v1, const int64_t v2) const -> Tuple
{
    const attribute::Accessor<int64_t> fv = create_const_accessor<int64_t>(m_fv_handle);
    auto& fv_base = create_base_accessor<int64_t>(m_fv_handle);
    for (int64_t fid = 0; fid < capacity(PrimitiveType::Triangle); ++fid) {
        Tuple face = face_tuple_from_id(fid);
        auto fv0 = fv.const_vector_attribute(face);
        int64_t local_vid1 = -1, local_vid2 = -1;
        for (int64_t i = 0; i < fv0.size(); ++i) {
            if (fv0[i] == v1) {
                local_vid1 = i;
            }
            if (fv0[i] == v2) {
                local_vid2 = i;
            }
        }
        if (local_vid1 != -1 && local_vid2 != -1) {
            return Tuple(
                local_vid1,
                (3 - local_vid1 - local_vid2) % 3,
                -1,
                fid);
        }
    }
    return Tuple();
}

auto DEBUG_TriMesh::face_tuple_from_vids(const int64_t v1, const int64_t v2, const int64_t v3) const
    -> Tuple
{
    const attribute::Accessor<int64_t> fv = create_const_accessor<int64_t>(m_fv_handle);
    auto& fv_base = create_base_accessor<int64_t>(m_fv_handle);
    for (const Tuple face : get_all(PrimitiveType::Triangle)) {
        auto fv0 = fv.const_vector_attribute(face);
        bool find_v1 = false, find_v2 = false, find_v3 = false;
        for (int64_t i = 0; i < fv0.size(); ++i) {
            if (fv0[i] == v1) {
                find_v1 = true;
            }
            if (fv0[i] == v2) {
                find_v2 = true;
            }
            if (fv0[i] == v3) {
                find_v3 = true;
            }
        }
        if (find_v1 && find_v2 && find_v3) {
            return face;
        }
    }
    return Tuple();
}

Tuple DEBUG_TriMesh::tuple_from_face_id(const int64_t fid) const
{
    return tuple_from_id(PrimitiveType::Triangle, fid);
}


const TypedAttributeHandle<int64_t>& DEBUG_TriMesh::f_handle(const PrimitiveType type) const
{
    switch (type) {
    case PrimitiveType::Vertex: return m_fv_handle;
    case PrimitiveType::Edge: return m_fe_handle;
    case PrimitiveType::Triangle: return m_ff_handle;
    default: throw std::runtime_error("Invalid PrimitiveType");
    }
}

const TypedAttributeHandle<int64_t>& DEBUG_TriMesh::vf_handle() const
{
    return m_vf_handle;
}

const TypedAttributeHandle<int64_t>& DEBUG_TriMesh::ef_handle() const
{
    return m_ef_handle;
}


void DEBUG_TriMesh::reserve_attributes(PrimitiveType type, int64_t size)
{
    Mesh::reserve_attributes(type, size);
}


auto DEBUG_TriMesh::get_tmoe(const Tuple& t)
    -> TriMeshOperationExecutor
{
    return TriMeshOperationExecutor(*this, t);
}
} // namespace wmtk::tests
