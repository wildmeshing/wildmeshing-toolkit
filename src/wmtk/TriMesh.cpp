#include "TriMesh.hpp"

#include <wmtk/utils/trimesh_topology_initialization.h>

namespace wmtk {
TriMesh::TriMesh()
    : m_vf_handle(register_attribute<long>("m_vf", PrimitiveType::Vertex, 1))
    , m_ef_handle(register_attribute<long>("m_ef", PrimitiveType::Edge, 1))
    , m_fv_handle(register_attribute<long>("m_fv", PrimitiveType::Face, 3))
    , m_fe_handle(register_attribute<long>("m_fe", PrimitiveType::Face, 3))
    , m_ff_handle(register_attribute<long>("m_ff", PrimitiveType::Face, 3))
{}

void TriMesh::split_edge(const Tuple& t) {}
void TriMesh::collapse_edge(const Tuple& t) {}

long TriMesh::id(const Tuple& tuple, const PrimitiveType& type) const
{
    switch (type) {
    case PrimitiveType::Vertex: {
        Accessor<long> fv_accessor = create_accessor<long>(m_fv_handle);
        auto fv = fv_accessor.vector_attribute(tuple);
        return fv(tuple.m_local_vid);
        break;
    }
    case PrimitiveType::Edge: {
        Accessor<long> fe_accessor = create_accessor<long>(m_fe_handle);
        auto fe = fe_accessor.vector_attribute(tuple);
        return fe(tuple.m_local_eid);
        break;
    }
    case PrimitiveType::Face: {
        return tuple.m_global_cid;
        break;
    }
    default: throw std::runtime_error("Tuple id: Invalid primitive type");
    }
}

Tuple TriMesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const
{
    throw "not implemeted";
    // bool ccw = is_ccw(tuple);
    // switch (type) {
    // case PrimitiveType::Vertex:
    //     return Tuple(
    //         (tuple.m_local_vid + ccw ? 1 : 2) % 3,
    //         tuple.m_local_eid,
    //         tuple.m_local_fid,
    //         tuple.m_global_cid,
    //         tuple.m_hash);

    // case PrimitiveType::Edge:
    //     return Tuple(
    //         tuple.m_local_vid,
    //         (tuple.m_local_eid + ccw ? 2 : 1) % 3,
    //         tuple.m_local_fid,
    //         tuple.m_global_cid,
    //         tuple.m_hash);
    // case PrimitiveType::Triangle: {
    //     long gvid = id(tuple, 0);
    //     long geid = id(tuple, 1);
    //     long gcid_new = m_ff[tuple.m_global_cid * 3 + tuple.m_local_eid];
    //     long lvid_new, leid_new;
    //     for (long i = 0; i < 3; ++i) {
    //         if (m_fe[gcid_new * 3 + i] == geid) {
    //             leid_new = m_fe[gcid_new * 3 + i];
    //         }
    //         if (m_fv[gcid_new * 3 + i] == gvid) {
    //             lvid_new = m_fv[gcid_new * 3 + i];
    //         }
    //     }
    //     return Tuple(lvid_new, leid_new, tuple.m_local_fid, gcid_new, tuple.m_hash);
    // }
    // default: throw std::runtime_error("Tuple switch: Invalid primitive type"); break;
    // }
}

bool TriMesh::is_ccw(const Tuple& tuple) const
{
    throw "not implemeted";
    Accessor<long> fv_accessor = create_accessor<long>(m_fv_handle);
    auto fv = fv_accessor.vector_attribute(tuple);
    if (fv((tuple.m_local_eid + 1) % 3) == id(tuple, PrimitiveType::Vertex))
        return true;
    else
        return false;
}

void TriMesh::initialize(
    Eigen::Ref<const RowVectors3l> FV,
    Eigen::Ref<const RowVectors3l> FE,
    Eigen::Ref<const RowVectors3l> FF,
    Eigen::Ref<const VectorXl> VF,
    Eigen::Ref<const VectorXl> EF)
{
    // reserve memory for attributes
    mesh_attributes_reserve(PrimitiveType::Face);
    // get Accessors for topology
    Accessor<long> fv_accessor = create_accessor<long>(m_fv_handle);
    Accessor<long> fe_accessor = create_accessor<long>(m_fe_handle);
    Accessor<long> ff_accessor = create_accessor<long>(m_ff_handle);
    Accessor<long> vf_accessor = create_accessor<long>(m_vf_handle);
    Accessor<long> ef_accessor = create_accessor<long>(m_ef_handle);
    // iterate over the matrices and fill attributes
    // m_fv
    for (long i = 0; i < FV.rows(); ++i) {
        for (long j = 0; j < FV.cols(); ++j) {
            long& v = fv_accessor.vector_attribute(i)(j);
            v = FV(i, j);
        }
    }
    // m_fe
    for (long i = 0; i < FE.rows(); ++i) {
        for (long j = 0; j < FE.cols(); ++j) {
            long& e = fe_accessor.vector_attribute(i)(j);
            e = FE(i, j);
        }
    }
    // m_ff
    for (long i = 0; i < FF.rows(); ++i) {
        for (long j = 0; j < FF.cols(); ++j) {
            long& f = ff_accessor.vector_attribute(i)(j);
            f = FF(i, j);
        }
    }
    // m_vf
    for (long i = 0; i < VF.rows(); ++i) {
        long& f = vf_accessor.scalar_attribute(i);
        f = VF(i);
    }
    // m_ef
    for (long i = 0; i < EF.rows(); ++i) {
        long& f = ef_accessor.scalar_attribute(i);
        f = EF(i);
    }
}

void TriMesh::initialize(Eigen::Ref<const RowVectors3l> F)
{
    RowVectors3l FE;
    RowVectors3l FF;
    VectorXl VF;
    VectorXl EF;

    trimesh_topology_initialization(F, FE, FF, VF, EF);

    initialize(F, FE, FF, VF, EF);
}




std::vector<Tuple> TriMesh::get_all(const PrimitiveType& type) const
{
    long simplex_index = get_simplex_dimension(type);
    Accessor<char> flag_accessor= get_accessor(m_simplex_flag_handles[simplex_index]);
    std::vector<Tuple> ret;
    long cap = capacity(type);
    ret.reserve(cap);
    for(size_t index = 0; index < cap; ++index) {
        if(!(flag_accessor.scalar_attribute(index) & 1)) {
        ret.emplace_back(tuple_from_id(simplex_index,index));
        }
    }
}

} // namespace wmtk
