#include "DEBUG_EdgeMesh.hpp"
#include <catch2/catch_test_macros.hpp>

namespace wmtk::tests {

DEBUG_EdgeMesh::DEBUG_EdgeMesh(const EdgeMesh& m)
    : EdgeMesh(m)
{}
DEBUG_EdgeMesh::DEBUG_EdgeMesh(EdgeMesh&& m)
    : EdgeMesh(std::move(m))
{}


bool DEBUG_EdgeMesh::operator==(const DEBUG_EdgeMesh& o) const
{
    throw std::runtime_error("This function is not tested yet");
    return static_cast<const EdgeMesh&>(*this) == static_cast<const EdgeMesh&>(o);
}
bool DEBUG_EdgeMesh::operator!=(const DEBUG_EdgeMesh& o) const
{
    throw std::runtime_error("This function is not tested yet");
    return !(*this == o);
}

void DEBUG_EdgeMesh::print_state() const
{
    throw std::runtime_error("This function is not implemented. maybe redundant");
}

void DEBUG_EdgeMesh::print_ve() const
{
    throw std::runtime_error(
        "this function was written in the style of DEBUG_TriMesh::print_vf() but was not tested "
        "yet");
    auto ev_accessor = create_base_accessor<long>(e_handle(PrimitiveType::Vertex));
    auto e_flag_accessor = get_flag_accessor(PrimitiveType::Edge);
    for (long id = 0; id < capacity(PrimitiveType::Edge); ++id) {
        auto ev = ev_accessor.const_vector_attribute(id);
        if (e_flag_accessor.const_scalar_attribute(tuple_from_id(PrimitiveType::Edge, id)) == 0) {
            std::cout << "edge " << id << " is deleted" << std::endl;
        } else {
            std::cout << ev(0) << " " << ev(1) << std::endl;
        }
    }
}

Eigen::Matrix<long, 2, 1> DEBUG_EdgeMesh::ev_from_eid(const long eid) const
{
    throw std::runtime_error("this function is never used");
    auto ev_accessor = create_base_accessor<long>(e_handle(PrimitiveType::Vertex));
    return ev_accessor.vector_attribute(eid);
}

auto DEBUG_EdgeMesh::edge_tuple_from_vids(const long v1, const long v2) const -> Tuple
{
    throw std::runtime_error("this function is never used");
    ConstAccessor<long> ev = create_accessor<long>(m_ev_handle);
    for (long eid = 0; eid < capacity(PrimitiveType::Edge); ++eid) {
        Tuple edge = edge_tuple_from_id(eid);
        auto ev0 = ev.const_vector_attribute(edge);
        long local_vid1 = -1, local_vid2 = -1;
        for (long i = 0; i < ev0.size(); ++i) {
            if (ev0[i] == v1) {
                local_vid1 = i;
            }
            if (ev0[i] == v2) {
                local_vid2 = i;
            }
        }
        if (local_vid1 != -1 && local_vid2 != -1) {
            return Tuple(local_vid1, -1, -1, eid, get_cell_hash_slow(eid));
        }
    }
    return Tuple();
}

auto DEBUG_EdgeMesh::tuple_from_edge_id(const long eid) const -> Tuple
{
    return tuple_from_id(PrimitiveType::Edge, eid);
}


const TypedAttributeHandle<long>& DEBUG_EdgeMesh::e_handle(const PrimitiveType type) const
{
    switch (type) {
    case PrimitiveType::Vertex: return m_ev_handle;
    case PrimitiveType::Edge: return m_ee_handle;
    case PrimitiveType::Face:;
    default: throw std::runtime_error("Invalid PrimitiveType");
    }
}

const TypedAttributeHandle<long>& DEBUG_EdgeMesh::ve_handle() const
{
    return m_ve_handle;
}

const TypedAttributeHandle<long>& DEBUG_EdgeMesh::ev_handle() const
{
    return m_ev_handle;
}


void DEBUG_EdgeMesh::reserve_attributes(PrimitiveType type, long size)
{
    Mesh::reserve_attributes(type, size);
}


long DEBUG_EdgeMesh::id(const Tuple& tuple, PrimitiveType type) const
{
    return EdgeMesh::id(tuple, type);
}
long DEBUG_EdgeMesh::id(const Simplex& s) const
{
    return id(s.tuple(), s.primitive_type());
}
Accessor<long> DEBUG_EdgeMesh::get_cell_hash_accessor()
{
    return EdgeMesh::get_cell_hash_accessor();
}
/**
 * @brief returns the EdgeMeshOperationExecutor
 */
auto DEBUG_EdgeMesh::get_emoe(const Tuple& t, Accessor<long>& hash_accessor)
    -> EdgeMeshOperationExecutor
{
    return EdgeMeshOperationExecutor(*this, t, hash_accessor);
}

bool DEBUG_EdgeMesh::is_simplex_deleted(PrimitiveType type, const long id) const
{
    const auto flag_accessor = get_flag_accessor(type);
    return flag_accessor.index_access().scalar_attribute(id) == 0;
}
} // namespace wmtk::tests
