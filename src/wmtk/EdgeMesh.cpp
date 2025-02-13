#include "EdgeMesh.hpp"


#include <wmtk/utils/edgemesh_topology_initialization.h>
#include <numeric>
#include <wmtk/utils/Logger.hpp>
namespace wmtk {
EdgeMesh::~EdgeMesh() = default;
EdgeMesh::EdgeMesh()
    : MeshCRTP<EdgeMesh>(1)
    , m_ve_handle(register_attribute_typed<int64_t>("m_ve", PrimitiveType::Vertex, 1, false, -1))
    , m_ev_handle(register_attribute_typed<int64_t>("m_ev", PrimitiveType::Edge, 2, false, -1))
    , m_ee_handle(register_attribute_typed<int64_t>("m_ee", PrimitiveType::Edge, 2, false, -1))
{}


bool EdgeMesh::is_boundary(PrimitiveType pt, const Tuple& tuple) const
{
    switch (pt) {
    case PrimitiveType::Vertex: return is_boundary_vertex(tuple);
    case PrimitiveType::Edge:
    case PrimitiveType::Triangle:
    case PrimitiveType::Tetrahedron:
    default: break;
    }
    assert(
        false); // "tried to compute the boundary of an edge mesh for an invalid simplex dimension"
    return false;
}

bool EdgeMesh::is_boundary_vertex(const Tuple& tuple) const
{
    assert(is_valid(tuple));
    const attribute::Accessor<int64_t> ee_accessor = create_const_accessor<int64_t>(m_ee_handle);
    return ee_accessor.const_vector_attribute<2>(tuple)(tuple.local_vid()) < 0;
}

Tuple EdgeMesh::switch_tuple(const Tuple& tuple, PrimitiveType type) const
{
    assert(is_valid(tuple));
    bool ccw = is_ccw(tuple);

    switch (type) {
    case PrimitiveType::Vertex:
        return Tuple(
            1 - tuple.local_vid(),
            tuple.local_eid(),
            tuple.local_fid(),
            tuple.global_cid());
    case PrimitiveType::Edge: {
        const int64_t gvid = id(tuple, PrimitiveType::Vertex);

        const attribute::Accessor<int64_t> ee_accessor =
            create_const_accessor<int64_t>(m_ee_handle);
        auto ee = ee_accessor.const_vector_attribute<2>(tuple);

        int64_t gcid_new = ee(tuple.local_vid());

        // This is for special case self-loop, just to make sure the local vid of the returned
        // tuple is the same as the input. (When doing double-switch this is needed)
        if (gcid_new == tuple.global_cid()) {
            return tuple;
        }

        int64_t lvid_new = -1;

        const attribute::Accessor<int64_t> ev_accessor =
            create_const_accessor<int64_t>(m_ev_handle);
        auto ev = ev_accessor.index_access().const_vector_attribute<2>(gcid_new);

        for (int64_t i = 0; i < 2; ++i) {
            if (ev(i) == gvid) {
                lvid_new = i;
                // break;
            }
        }
        assert(lvid_new != -1);


        const Tuple res(lvid_new, tuple.local_eid(), tuple.local_fid(), gcid_new);
        assert(is_valid(res));
        return res;
    }
    case PrimitiveType::Triangle:
    case PrimitiveType::Tetrahedron:
    default: assert(false); // "Tuple switch: Invalid primitive type"
    }

    return Tuple();
}

bool EdgeMesh::is_ccw(const Tuple& tuple) const
{
    assert(is_valid(tuple));
    return tuple.local_vid() == 0;
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
    attribute::Accessor<int64_t> ev_accessor = create_accessor<int64_t>(m_ev_handle);
    attribute::Accessor<int64_t> ee_accessor = create_accessor<int64_t>(m_ee_handle);
    attribute::Accessor<int64_t> ve_accessor = create_accessor<int64_t>(m_ve_handle);

    attribute::FlagAccessor<EdgeMesh> v_flag_accessor = get_flag_accessor(PrimitiveType::Vertex);
    attribute::FlagAccessor<EdgeMesh> e_flag_accessor = get_flag_accessor(PrimitiveType::Edge);

    // iterate over the matrices and fill attributes

    for (int64_t i = 0; i < capacity(PrimitiveType::Edge); ++i) {
        ev_accessor.index_access().vector_attribute<2>(i) = EV.row(i).transpose();
        ee_accessor.index_access().vector_attribute<2>(i) = EE.row(i).transpose();

        e_flag_accessor.index_access().activate(i);
    }
    // m_ve
    for (int64_t i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        ve_accessor.index_access().scalar_attribute(i) = VE(i);
        v_flag_accessor.index_access().activate(i);
    }
}

void EdgeMesh::initialize(Eigen::Ref<const RowVectors2l> E, bool is_free)
{
    this->m_is_free = is_free;
    auto [EE, VE] = edgemesh_topology_initialization(E);
    if (is_free) {
        EE.setConstant(-1);
    }
    initialize(E, EE, VE);
}
void EdgeMesh::initialize_free(int64_t count)
{
    RowVectors2l S(count, 2);
    std::iota(S.data(), S.data() + S.size(), int64_t(0));
    initialize(S, true);
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
    case PrimitiveType::Triangle: {
        throw std::runtime_error("no tet tuple supported for edgemesh");
        break;
    }
    case PrimitiveType::Tetrahedron: {
        throw std::runtime_error("no tet tuple supported for edgemesh");
        break;
    }
    default: assert(false); //"Invalid primitive type"
    }

    return Tuple();
}

Tuple EdgeMesh::vertex_tuple_from_id(int64_t id) const
{
    const attribute::Accessor<int64_t> ve_accessor = create_const_accessor<int64_t>(m_ve_handle);
    auto e = ve_accessor.index_access().const_scalar_attribute(id);
    const attribute::Accessor<int64_t> ev_accessor = create_const_accessor<int64_t>(m_ev_handle);
    auto ev = ev_accessor.index_access().const_vector_attribute<2>(e);
    for (int64_t i = 0; i < 2; ++i) {
        if (ev(i) == id) {
            Tuple v_tuple = Tuple(i, -1, -1, e);
            return v_tuple;
        }
    }
    assert(false); // "vertex_tuple_from_id failed"

    return Tuple();
}

Tuple EdgeMesh::edge_tuple_from_id(int64_t id) const
{
    Tuple e_tuple = Tuple(0, -1, -1, id);

    assert(is_valid(e_tuple));
    return e_tuple;
}

Tuple EdgeMesh::tuple_from_global_ids(int64_t eid, int64_t vid) const
{
    const attribute::Accessor<int64_t> ev_accessor = create_const_accessor<int64_t>(m_ev_handle);
    auto ev = ev_accessor.index_access().const_vector_attribute<2>(eid);

    int64_t lvid = -1;

    for (int j = 0; j < 2; ++j) {
        if (ev(j) == vid) {
            lvid = j;
        }
    }
    assert(lvid != -1);

    return Tuple(lvid, -1, -1, eid);
}


bool EdgeMesh::is_valid(const Tuple& tuple) const
{
    if (!Mesh::is_valid(tuple)) {
        return false;
    }

    const bool is_connectivity_valid = tuple.local_vid() >= 0 && tuple.global_cid() >= 0;
    if (!is_connectivity_valid) {
#if !defined(NDEBUG)
        logger().debug(
            "tuple.local_vid()={} >= 0"
            " tuple.global_cid()={} >= 0",
            tuple.local_vid(),
            tuple.global_cid());
        assert(tuple.local_vid() >= 0);
        assert(tuple.global_cid() >= 0);
#endif
        return false;
    }
    return true;
}

bool EdgeMesh::is_connectivity_valid() const
{
    // get accessors for topology
    const attribute::Accessor<int64_t> ev_accessor = create_const_accessor<int64_t>(m_ev_handle);
    const attribute::Accessor<int64_t> ee_accessor = create_const_accessor<int64_t>(m_ee_handle);
    const attribute::Accessor<int64_t> ve_accessor = create_const_accessor<int64_t>(m_ve_handle);
    const attribute::FlagAccessor<EdgeMesh> v_flag_accessor =
        get_flag_accessor(PrimitiveType::Vertex);
    const attribute::FlagAccessor<EdgeMesh> e_flag_accessor =
        get_flag_accessor(PrimitiveType::Edge);

    // VE and EV
    for (int64_t i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        if (!v_flag_accessor.index_access().is_active(i)) {
            continue;
        }
        int cnt = 0;
        for (int64_t j = 0; j < 2; ++j) {
            if (ev_accessor.index_access().const_vector_attribute<2>(
                    ve_accessor.index_access().const_scalar_attribute(i))(j) == i) {
                cnt++;
            }
        }
        if (cnt == 0) {
            int64_t idx = ve_accessor.index_access().const_scalar_attribute(i);
            wmtk::logger().error(
                "EV[VE[{}]={},:]] ({}) = doesn't contain {}",
                i,
                idx,
                fmt::join(ev_accessor.index_access().const_vector_attribute<2>(idx), ","),
                i);
            return false;
        }
    }

    // EV and EE
    for (int64_t i = 0; i < capacity(PrimitiveType::Edge); ++i) {
        if (!e_flag_accessor.index_access().is_active(i)) {
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

std::vector<Tuple> EdgeMesh::orient_vertices(const Tuple& tuple) const
{
    int64_t cid = tuple.global_cid();
    return {Tuple(0, -1, -1, cid), Tuple(1, -1, -1, cid)};
}


} // namespace wmtk
