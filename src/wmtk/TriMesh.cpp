#include "TriMesh.hpp"

#include <wmtk/utils/trimesh_topology_initialization.h>
#include <wmtk/autogen/autogenerated_2d_tables.hpp>

namespace wmtk {
TriMesh::TriMesh()
    : Mesh(3)
    , m_vf_handle(register_attribute<long>("m_vf", PrimitiveType::Vertex, 1))
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
        ConstAccessor<long> fv_accessor = create_accessor<long>(m_fv_handle);
        auto fv = fv_accessor.vector_attribute(tuple);
        return fv(tuple.m_local_vid);
        break;
    }
    case PrimitiveType::Edge: {
        ConstAccessor<long> fe_accessor = create_accessor<long>(m_fe_handle);
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

    // bool ccw = is_ccw(tuple);
    // int offset = (tuple.m_local_vid*3 + tuple.m_local_eid);

    // switch (type) {
    // case PrimitiveType::Vertex:
    //     return Tuple(
    //         wmtk::autogen::2d_tuple_table_vertex[offset][0],
    //         wmtk::autogen::2d_tuple_table_vertex[offset][1],
    //         tuple.m_local_fid,
    //         tuple.m_global_cid,
    //         tuple.m_hash);
Tuple TriMesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const
{
    bool ccw = is_ccw(tuple);
    int offset = (tuple.m_local_vid * 3 + tuple.m_local_eid);

    switch (type) {
    case PrimitiveType::Vertex:
        return Tuple(
            wmtk::autogen::auto_2d_table_vertex[offset][0],
            wmtk::autogen::auto_2d_table_vertex[offset][1],
            tuple.m_local_fid,
            tuple.m_global_cid,
            tuple.m_hash);

    case PrimitiveType::Edge:
        return Tuple(
            wmtk::autogen::auto_2d_table_edge[offset][0],
            wmtk::autogen::auto_2d_table_edge[offset][1],
            tuple.m_local_fid,
            tuple.m_global_cid,
            tuple.m_hash);
    case PrimitiveType::Face: {
        long gvid = id(tuple, PrimitiveType::Vertex);
        long geid = id(tuple, PrimitiveType::Edge);
        ConstAccessor<long> ff_accessor = create_accessor<long>(m_ff_handle);
        auto ff = ff_accessor.vector_attribute(tuple);
        long gcid_new = ff(tuple.m_local_eid);
        long lvid_new, leid_new;
        ConstAccessor<long> fv_accessor = create_accessor<long>(m_fv_handle);
        auto fv = fv_accessor.vector_attribute(gcid_new);
        ConstAccessor<long> fe_accessor = create_accessor<long>(m_fe_handle);
        auto fe = fe_accessor.vector_attribute(gcid_new);
        for (long i = 0; i < 3; ++i) {
            if (fe(i) == geid) {
                leid_new = fe(i);
            }
            if (fv(i) == gvid) {
                lvid_new = fv(i);
            }
        }
        return Tuple(lvid_new, leid_new, tuple.m_local_fid, gcid_new, tuple.m_hash);
    }
    default: throw std::runtime_error("Tuple switch: Invalid primitive type"); break;
    }
}

bool TriMesh::is_ccw(const Tuple& tuple) const
{
    ConstAccessor<long> fv_accessor = create_accessor<long>(m_fv_handle);
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

    std::vector<long> cap{
        static_cast<long>(FF.rows()),
        static_cast<long>(VF.rows()),
        static_cast<long>(EF.rows())};
    set_capacities(cap);
    reserve_attributes_to_fit();

    // get Accessors for topology
    Accessor<long> fv_accessor = create_accessor<long>(m_fv_handle);
    Accessor<long> fe_accessor = create_accessor<long>(m_fe_handle);
    Accessor<long> ff_accessor = create_accessor<long>(m_ff_handle);
    Accessor<long> vf_accessor = create_accessor<long>(m_vf_handle);
    Accessor<long> ef_accessor = create_accessor<long>(m_ef_handle);
    // iterate over the matrices and fill attributes
    // m_fv
    for (long i = 0; i < FV.rows(); ++i) {
        fv_accessor.vector_attribute(i) = FV.row(i).transpose();
    }
    // m_fe
    for (long i = 0; i < FE.rows(); ++i) {
        fe_accessor.vector_attribute(i) = FE.row(i).transpose();
    }
    // m_ff
    for (long i = 0; i < FF.rows(); ++i) {
        ff_accessor.vector_attribute(i) = FF.row(i).transpose();
    }
    // m_vf
    for (long i = 0; i < VF.rows(); ++i) {
        vf_accessor.scalar_attribute(i) = VF(i);
    }
    // m_ef
    for (long i = 0; i < EF.rows(); ++i) {
        ef_accessor.scalar_attribute(i) = EF(i);
    }
}

void TriMesh::initialize(Eigen::Ref<const RowVectors3l> F)
{
    auto [FE, FF, VF, EF] = trimesh_topology_initialization(F);
    initialize(F, FE, FF, VF, EF);
}

std::vector<Tuple> TriMesh::get_all(const PrimitiveType& type) const
{
    switch (type) {
    case PrimitiveType::Vertex: return get_all_vertices();
    case PrimitiveType::Edge: return get_all_edges(); break;
    case PrimitiveType::Face: return get_all_faces(); break;
    case PrimitiveType::Tetrahedron:
    default: throw std::runtime_error("Invalid primitive type");
    }
}

std::vector<Tuple> TriMesh::get_all_vertices() const
{
    throw "not implemented";
}
std::vector<Tuple> TriMesh::get_all_edges() const
{
    throw "not implemented";
}
std::vector<Tuple> TriMesh::get_all_faces() const
{
    throw "not implemented";
}
} // namespace wmtk
