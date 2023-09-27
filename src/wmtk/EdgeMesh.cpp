#include "EdgeMesh.hpp"
namespace wmtk {
EdgeMesh::EdgeMesh()
    : Mesh(1)
    , m_ve_handle(register_attribute<long>("m_ve", PrimitiveType::Vertex, 1))
    , m_ev_handle(register_attribute<long>("m_ev", PrimitiveType::Edge, 2))
    , m_ee_handle(register_attribute<long>("m_ee", PrimitiveType::Edge, 2))
{}
// EdgeMesh::EdgeMesh(const EdgeMesh&& o) = default;
EdgeMesh::EdgeMesh(EdgeMesh&& o) = default;
EdgeMesh& EdgeMesh::operator=(const EdgeMesh& o) = default;
EdgeMesh& EdgeMesh::operator=(EdgeMesh&& o) = default;

Tuple EdgeMesh::tuple_from_id(const PrimitiveType type, const long gid) const
{
    throw std::runtime_error("this function has not been tested! -- EdgeMesh.hpp/EdgeMesh.cpp");
    switch (type) {
    case PrimitiveType::Vertex: {
        return vertex_tuple_from_id(gid);
    }
    case PrimitiveType::Edge: {
        return edge_tuple_from_id(gid);
    }
    case PrimitiveType::Face: {
        throw std::runtime_error("no face tuple supported for edgemesh");
        break;
    }
    case PrimitiveType::Tetrahedron: {
        throw std::runtime_error("no tet tuple supported for edgemesh");
        break;
    }
    default: throw std::runtime_error("Invalid primitive type"); break;
    }
}

Tuple EdgeMesh::switch_tuple(const Tuple& tuple, PrimitiveType type) const
{
    throw std::runtime_error("this function has not been tested! -- EdgeMesh.hpp/EdgeMesh.cpp");
    assert(is_valid_slow(tuple));
    bool ccw = is_ccw(tuple);

    //
    switch (type) {
    case PrimitiveType::Vertex:
        return Tuple(
            (tuple.m_local_vid + 1) % (long)2,
            tuple.m_local_eid,
            tuple.m_local_fid,
            tuple.m_global_cid,
            tuple.m_hash);
    case PrimitiveType::Edge:
    // don't know what is the result of the edge's switch, thought we just need vertex_switch?
    case PrimitiveType::Face:
    case PrimitiveType::Tetrahedron:
    default: throw std::runtime_error("Tuple switch: Invalid primitive type"); break;
    }
}
bool EdgeMesh::is_ccw(const Tuple&) const
{
    throw("this function has not been tested! -- EdgeMesh.hpp/EdgeMesh.cpp");
    // we don't need the oriented edge, so it should always be true
    return true;
}
bool EdgeMesh::is_boundary(const Tuple& tuple) const
{
    throw("this function has not been tested! -- EdgeMesh.hpp/EdgeMesh.cpp");
    assert(is_valid_slow(tuple));
    // fix an edge and a vertex, then compute the neighbour edge
    ConstAccessor<long> ee_accessor = create_const_accessor<long>(m_ee_handle);
    return ee_accessor.vector_attribute(tuple)(tuple.m_local_vid) < 0;
}

bool EdgeMesh::is_boundary_vertex(const Tuple& tuple) const
{
    throw("this function has not been implemented! -- EdgeMesh.hpp/EdgeMesh.cpp");
    // boudnary_vertex is a vertex with only one degree
    assert(is_valid_slow(tuple));
    ConstAccessor<long> ve_accessor = create_const_accessor<long>(m_ve_handle);
    // not sure if it works.
    // I think the main problem is I still not sure about the mechanisim of the "id" and "e_tuple"
    // funtions.
    auto e = ve_accessor.index_access().scalar_attribute(tuple.m_local_vid);
    ConstAccessor<long> ev_accessor = create_const_accessor<long>(m_ev_handle);
    auto ev = ev_accessor.index_access().vector_attribute(e);
    // tuple_from_id(,);
    return false;
}

void EdgeMesh::initialize(Eigen::Ref<const RowVectors2l> E)
{
    // resize the EE and VE
    RowVectors2l EE;
    RowVectors2l VE;
    EE.resize(E.rows(), 2);
    long max_vid = -1;
    for (size_t i = 0; i < E.rows(); i++) {
        EE(i, 0) = -1;
        EE(i, 1) = -1;
        max_vid = std::max(max_vid, E(i, 0));
        max_vid = std::max(max_vid, E(i, 1));
    }
    VE.resize(max_vid + 1, 2);
    for (size_t i = 0; i < VE.rows(); i++) {
        VE(i, 0) = -1;
        VE(i, 1) = -1;
    }

    // compute VE
    for (size_t i = 0; i < E.rows(); i++) {
        long vid0, vid1;
        vid0 = E(i, 0);
        vid1 = E(i, 1);
        if (VE(vid0, 0) == -1) {
            VE(vid0, 0) = i;
        } else if (VE(vid0, 1) == -1) {
            VE(vid0, 1) = i;
        }
        if (VE(vid1, 0) == -1) {
            VE(vid1, 0) = i;
        } else if (VE(vid1, 1) == -1) {
            VE(vid1, 1) = i;
        }
    }

    // compute EE
    for (size_t i = 0; i < VE.rows(); i++) {
        long eid0 = VE(i, 0);
        long eid1 = VE(i, 1);
        if (EE(eid0, 0) == -1) {
            EE(eid0, 0) = eid1;
        } else if (EE(eid1, 0) == -1) {
            EE(eid0, 1) = eid1;
        }
        if (EE(eid0, 0) == -1) {
            EE(eid1, 0) = eid0;
        } else if (EE(eid1, 0) == -1) {
            EE(eid1, 1) = eid0;
        }
    }

    initialize(E, EE, VE);
}

void EdgeMesh::initialize(
    Eigen::Ref<const RowVectors2l> EV,
    Eigen::Ref<const RowVectors2l> EE,
    Eigen::Ref<const RowVectors2l> VE)
{
    throw("this function has not been tested! -- EdgeMesh.hpp/EdgeMesh.cpp");
    std::vector<long> cap{
        static_cast<long>(EV.rows()),
        static_cast<long>(EE.rows()),
        static_cast<long>(VE.rows())};

    set_capacities(cap);

    // get Accessors for topology
    Accessor<long> ev_accessor = create_accessor<long>(m_ev_handle);
    Accessor<long> ve_accessor = create_accessor<long>(m_ve_handle);
    Accessor<long> ee_accessor = create_accessor<long>(m_ee_handle);

    Accessor<char> v_flag_accessor = get_flag_accessor(PrimitiveType::Vertex);
    Accessor<char> e_flag_accessor = get_flag_accessor(PrimitiveType::Edge);

    // wait to add
    // ...
    // iterate over the matrices and fill attributes
    for (long i = 0; i < capacity(PrimitiveType::Edge); ++i) {
        ev_accessor.index_access().vector_attribute(i) = EV.row(i).transpose();
        ee_accessor.index_access().vector_attribute(i) = EE.row(i).transpose();

        e_flag_accessor.index_access().scalar_attribute(i) |= 0x1;
    }

    // m_vf
    for (long i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        ve_accessor.index_access().vector_attribute(i) = VE.row(i).transpose();
        // e_flag_accessor.index_access().scalar_attribute(i) |= 0x1;
    }
}

bool EdgeMesh::is_valid(const Tuple& tuple, ConstAccessor<long>& hash_accessor) const
{
    throw("this function has not been implemented! -- EdgeMesh.hpp/EdgeMesh.cpp");
    // copied from PointMesh.cpp
    if (tuple.is_null()) return false;
    return true;
    return Mesh::is_hash_valid(tuple, hash_accessor);
}

long EdgeMesh::id(const Tuple& tuple, PrimitiveType type) const
{
    throw("this function has not been tested! -- EdgeMesh.hpp/EdgeMesh.cpp");
    switch (type) {
    case PrimitiveType::Vertex: {
        ConstAccessor<long> ev_accessor = create_const_accessor<long>(m_ev_handle);
        auto ev = ev_accessor.vector_attribute(tuple);
        return ev(tuple.m_local_vid);
    }
    case PrimitiveType::Edge: {
        return tuple.m_global_cid;
    }
    case PrimitiveType::Face:
    case PrimitiveType::Tetrahedron:
    default: throw std::runtime_error("Tuple id: Invalid primitive type");
    }
}

Tuple EdgeMesh::vertex_tuple_from_id(long id) const
{
    throw("this function has not been tested! -- EdgeMesh.hpp/EdgeMesh.cpp");
    ConstAccessor<long> ve_accessor = create_const_accessor<long>(m_ve_handle);
    // confused about this "id", does this mean the id of vertex or the edge? I refer to the
    // TriMesh.cpp(from line 243)
    auto e = ve_accessor.index_access().scalar_attribute(id);
    ConstAccessor<long> ev_accessor = create_const_accessor<long>(m_ev_handle);
    auto ev = ev_accessor.index_access().vector_attribute(e);
    for (long i = 0; i < 2; ++i) {
        if (ev(i) == id) {
            // assert(autogen::auto_2d_table_complete_vertex[i][0] == i);
            // do not know if I use this function right.
            Tuple v_tuple = Tuple(
                i,
                -1,
                -1,
                e,
                get_cell_hash_slow(
                    e)); // TODO replace by function that takes hash accessor as parameter
            assert(is_ccw(v_tuple)); // is_ccw also checks for validity
            return v_tuple;
        }
    }
    throw std::runtime_error("vertex_tuple_from_id failed");
}

Tuple EdgeMesh::edge_tuple_from_id(long id) const
{
    throw("this function has not been tested! -- EdgeMesh.hpp/EdgeMesh.cpp");
    Tuple e_tuple = Tuple(-1, -1, -1, id, get_cell_hash_slow(id));
    assert(is_ccw(e_tuple));
    assert(is_valid_slow(e_tuple));
    return e_tuple;
    throw std::runtime_error("edge_tuple_from_id failed");
}
} // namespace wmtk
