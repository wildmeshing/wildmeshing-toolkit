#include "EdgeMesh.hpp"


#include <wmtk/utils/edgemesh_topology_initialization.h>
#include <wmtk/utils/Logger.hpp>
namespace wmtk {
EdgeMesh::EdgeMesh()
    : Mesh(1)
    , m_ve_handle(
          register_attribute_nomesh<int64_t>("m_ve", PrimitiveType::Vertex, 1, false, 0, false))
    , m_ev_handle(register_attribute_nomesh<int64_t>("m_ev", PrimitiveType::Edge, 2, false, 0, false))
    , m_ee_handle(register_attribute_nomesh<int64_t>("m_ee", PrimitiveType::Edge, 2, false, 0, false))
{}
EdgeMesh::EdgeMesh(const EdgeMesh& o) = default;
EdgeMesh::EdgeMesh(EdgeMesh&& o) = default;
EdgeMesh& EdgeMesh::operator=(const EdgeMesh& o) = default;
EdgeMesh& EdgeMesh::operator=(EdgeMesh&& o) = default;

int64_t EdgeMesh::id(const Tuple& tuple, PrimitiveType type) const
{
    switch (type) {
    case PrimitiveType::Vertex: {
        ConstAccessor<int64_t> ev_accessor = create_const_accessor<int64_t>(m_ev_handle);
        auto ev = ev_accessor.vector_attribute(tuple);
        return ev(tuple.m_local_vid);
    }
    case PrimitiveType::Edge: {
        return tuple.m_global_cid;
    }
    case PrimitiveType::HalfEdge:
    case PrimitiveType::Face:
    case PrimitiveType::Tetrahedron:
    default: throw std::runtime_error("Tuple id: Invalid primitive type");
    }
}

bool EdgeMesh::is_boundary(const Tuple& tuple, PrimitiveType pt) const
{
    switch (pt) {
    case PrimitiveType::Vertex: return is_boundary_vertex(tuple);
    case PrimitiveType::Edge:
    case PrimitiveType::Face:
    case PrimitiveType::Tetrahedron:
    case PrimitiveType::HalfEdge:
    default: break;
    }
    throw std::runtime_error(
        "tried to compute the boundary of an edge mesh for an invalid simplex dimension");
    return false;
}

bool EdgeMesh::is_boundary_vertex(const Tuple& tuple) const
{
    assert(is_valid_slow(tuple));
    ConstAccessor<int64_t> ee_accessor = create_const_accessor<int64_t>(m_ee_handle);
    return ee_accessor.vector_attribute(tuple)(tuple.m_local_vid) < 0;
}

Tuple EdgeMesh::switch_tuple(const Tuple& tuple, PrimitiveType type) const
{
    assert(is_valid_slow(tuple));
    bool ccw = is_ccw(tuple);

    switch (type) {
    case PrimitiveType::Vertex:
        return Tuple(
            1 - tuple.m_local_vid,
            tuple.m_local_eid,
            tuple.m_local_fid,
            tuple.m_global_cid,
            tuple.m_hash);
    case PrimitiveType::Edge: {
        const int64_t gvid = id(tuple, PrimitiveType::Vertex);

        ConstAccessor<int64_t> ee_accessor = create_const_accessor<int64_t>(m_ee_handle);
        auto ee = ee_accessor.vector_attribute(tuple);

        int64_t gcid_new = ee(tuple.m_local_vid);

        // This is for special case self-loop, just to make sure the local vid of the returned
        // tuple is the same as the input. (When doing double-switch this is needed)
        if (gcid_new == tuple.m_global_cid) {
            return tuple;
        }

        int64_t lvid_new = -1;

        ConstAccessor<int64_t> ev_accessor = create_const_accessor<int64_t>(m_ev_handle);
        auto ev = ev_accessor.index_access().vector_attribute(gcid_new);

        for (int64_t i = 0; i < 2; ++i) {
            if (ev(i) == gvid) {
                lvid_new = i;
                // break;
            }
        }
        assert(lvid_new != -1);

        ConstAccessor<int64_t> hash_accessor = get_const_cell_hash_accessor();

        const Tuple res(
            lvid_new,
            tuple.m_local_eid,
            tuple.m_local_fid,
            gcid_new,
            get_cell_hash(gcid_new, hash_accessor));
        assert(is_valid(res, hash_accessor));
        return res;
    }
    case PrimitiveType::HalfEdge:
    case PrimitiveType::Face:
    case PrimitiveType::Tetrahedron:
    default: throw std::runtime_error("Tuple switch: Invalid primitive type"); break;
    }
}

bool EdgeMesh::is_ccw(const Tuple& tuple) const
{
    assert(is_valid_slow(tuple));
    return tuple.m_local_vid == 0;
}

void EdgeMesh::initialize(
    Eigen::Ref<const RowVectors2l> EV,
    Eigen::Ref<const RowVectors2l> EE,
    Eigen::Ref<const VectorXl> VE)
{
    // reserve memory for attributes

    std::vector<int64_t> cap{static_cast<int64_t>(VE.rows()), static_cast<int64_t>(EE.rows())};

    set_capacities(cap);

    // get accessors for topology
    Accessor<int64_t> ev_accessor = create_accessor<int64_t>(m_ev_handle);
    Accessor<int64_t> ee_accessor = create_accessor<int64_t>(m_ee_handle);
    Accessor<int64_t> ve_accessor = create_accessor<int64_t>(m_ve_handle);

    Accessor<char> v_flag_accessor = get_flag_accessor(PrimitiveType::Vertex);
    Accessor<char> e_flag_accessor = get_flag_accessor(PrimitiveType::Edge);

    // iterate over the matrices and fill attributes

    for (int64_t i = 0; i < capacity(PrimitiveType::Edge); ++i) {
        ev_accessor.index_access().vector_attribute(i) = EV.row(i).transpose();
        ee_accessor.index_access().vector_attribute(i) = EE.row(i).transpose();

        e_flag_accessor.index_access().scalar_attribute(i) |= 0x1;
    }
    // m_ve
    for (int64_t i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        ve_accessor.index_access().scalar_attribute(i) = VE(i);
        v_flag_accessor.index_access().scalar_attribute(i) |= 0x1;
    }
}

void EdgeMesh::initialize(Eigen::Ref<const RowVectors2l> E)
{
    auto [EE, VE] = edgemesh_topology_initialization(E);
    initialize(E, EE, VE);
}

Tuple EdgeMesh::tuple_from_id(const PrimitiveType type, const int64_t gid) const
{
    switch (type) {
    case PrimitiveType::Vertex: {
        return vertex_tuple_from_id(gid);
    }
    case PrimitiveType::Edge: {
        return edge_tuple_from_id(gid);
    }
    case PrimitiveType::Face: {
        throw std::runtime_error("no tet tuple supported for edgemesh");
        break;
    }
    case PrimitiveType::HalfEdge:
    case PrimitiveType::Tetrahedron: {
        throw std::runtime_error("no tet tuple supported for edgemesh");
        break;
    }
    default: throw std::runtime_error("Invalid primitive type"); break;
    }
}

Tuple EdgeMesh::vertex_tuple_from_id(int64_t id) const
{
    ConstAccessor<int64_t> ve_accessor = create_const_accessor<int64_t>(m_ve_handle);
    auto e = ve_accessor.index_access().scalar_attribute(id);
    ConstAccessor<int64_t> ev_accessor = create_const_accessor<int64_t>(m_ev_handle);
    auto ev = ev_accessor.index_access().vector_attribute(e);
    for (int64_t i = 0; i < 2; ++i) {
        if (ev(i) == id) {
            Tuple v_tuple = Tuple(i, -1, -1, e, get_cell_hash_slow(e));
            return v_tuple;
        }
    }
    throw std::runtime_error("vertex_tuple_from_id failed");
}

Tuple EdgeMesh::edge_tuple_from_id(int64_t id) const
{
    Tuple e_tuple = Tuple(0, -1, -1, id, get_cell_hash_slow(id));

    assert(is_valid_slow(e_tuple));
    return e_tuple;
}

Tuple EdgeMesh::tuple_from_global_ids(int64_t eid, int64_t vid) const
{
    ConstAccessor<int64_t> ev_accessor = create_const_accessor<int64_t>(m_ev_handle);
    auto ev = ev_accessor.index_access().vector_attribute(eid);

    int64_t lvid = -1;

    for (int j = 0; j < 2; ++j) {
        if (ev(j) == vid) {
            lvid = j;
        }
    }
    assert(lvid != -1);

    return Tuple(
        lvid,
        -1,
        -1,
        eid,
        get_cell_hash_slow(eid)); // TODO replace by function that takes hash accessor as parameter
}


bool EdgeMesh::is_valid(const Tuple& tuple, ConstAccessor<int64_t>& hash_accessor) const
{
    if (tuple.is_null()) return false;

    if (tuple.m_local_vid < 0 || tuple.m_global_cid < 0) return false;

    return Mesh::is_hash_valid(tuple, hash_accessor);
}

bool EdgeMesh::is_connectivity_valid() const
{
    // get accessors for topology
    ConstAccessor<int64_t> ev_accessor = create_const_accessor<int64_t>(m_ev_handle);
    ConstAccessor<int64_t> ee_accessor = create_const_accessor<int64_t>(m_ee_handle);
    ConstAccessor<int64_t> ve_accessor = create_const_accessor<int64_t>(m_ve_handle);
    ConstAccessor<char> v_flag_accessor = get_flag_accessor(PrimitiveType::Vertex);
    ConstAccessor<char> e_flag_accessor = get_flag_accessor(PrimitiveType::Edge);

    // VE and EV
    for (int64_t i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        if (v_flag_accessor.index_access().scalar_attribute(i) == 0) {
            wmtk::logger().debug("Vertex {} is deleted", i);
            continue;
        }
        int cnt = 0;
        for (int64_t j = 0; j < 2; ++j) {
            if (ev_accessor.index_access().vector_attribute(
                    ve_accessor.index_access().scalar_attribute(i))[j] == i) {
                cnt++;
            }
        }
        if (cnt == 0) {
            return false;
        }
    }

    // EV and EE
    for (int64_t i = 0; i < capacity(PrimitiveType::Edge); ++i) {
        if (e_flag_accessor.index_access().scalar_attribute(i) == 0) {
            wmtk::logger().debug("Edge {} is deleted", i);
            continue;
        }
        // TODO: need to handle cornor case (self-loop)
    }

    return true;
}

std::vector<std::vector<TypedAttributeHandle<int64_t>>> EdgeMesh::connectivity_attributes() const
{
    std::vector<std::vector<TypedAttributeHandle<int64_t>>> handles(2);

    handles[1].push_back(m_ve_handle);
    handles[1].push_back(m_ee_handle);
    handles[0].push_back(m_ev_handle);

    return handles;
}


} // namespace wmtk
