#include "TriMesh.hpp"

#include <wmtk/utils/trimesh_topology_initialization.h>
#include <numeric>
#include <wmtk/autogen/tri_mesh/is_ccw.hpp>
#include <wmtk/autogen/tri_mesh/local_switch_tuple.hpp>
#include <wmtk/utils/Logger.hpp>
#include "wmtk/autogen/tri_mesh/get_tuple_from_simplex_local_id.hpp"

namespace wmtk {

TriMesh::~TriMesh() = default;
TriMesh::TriMesh()
    : MeshCRTP<TriMesh>(2)
    , m_vf_handle(register_attribute_typed<int64_t>("m_vf", PrimitiveType::Vertex, 1, false, -1))
    , m_ef_handle(register_attribute_typed<int64_t>("m_ef", PrimitiveType::Edge, 1, false, -1))
    , m_fv_handle(register_attribute_typed<int64_t>("m_fv", PrimitiveType::Triangle, 3, false, -1))
    , m_fe_handle(register_attribute_typed<int64_t>("m_fe", PrimitiveType::Triangle, 3, false, -1))
    , m_ff_handle(register_attribute_typed<int64_t>("m_ff", PrimitiveType::Triangle, 3, false, -1))
{
    make_cached_accessors();
}

void TriMesh::make_cached_accessors()
{
    m_vf_accessor = std::make_unique<attribute::Accessor<int64_t, TriMesh>>(*this, m_vf_handle);
    m_ef_accessor = std::make_unique<attribute::Accessor<int64_t, TriMesh>>(*this, m_ef_handle);
    m_fv_accessor = std::make_unique<attribute::Accessor<int64_t, TriMesh>>(*this, m_fv_handle);
    m_fe_accessor = std::make_unique<attribute::Accessor<int64_t, TriMesh>>(*this, m_fe_handle);
    m_ff_accessor = std::make_unique<attribute::Accessor<int64_t, TriMesh>>(*this, m_ff_handle);
}

TriMesh::TriMesh(TriMesh&& o)
    : MeshCRTP<TriMesh>(std::move(o))
{
    m_vf_handle = o.m_vf_handle;
    m_ef_handle = o.m_ef_handle;
    m_fv_handle = o.m_fv_handle;
    m_fe_handle = o.m_fe_handle;
    m_ff_handle = o.m_ff_handle;

    make_cached_accessors();
}
TriMesh& TriMesh::operator=(TriMesh&& o)
{
    Mesh::operator=(std::move(o));
    m_vf_handle = o.m_vf_handle;
    m_ef_handle = o.m_ef_handle;
    m_fv_handle = o.m_fv_handle;
    m_fe_handle = o.m_fe_handle;
    m_ff_handle = o.m_ff_handle;

    make_cached_accessors();
    return *this;
}


bool TriMesh::is_boundary(PrimitiveType pt, const Tuple& tuple) const
{
    switch (pt) {
    case PrimitiveType::Vertex: return is_boundary_vertex(tuple);
    case PrimitiveType::Edge: return is_boundary_edge(tuple);
    case PrimitiveType::Triangle:
    case PrimitiveType::Tetrahedron:
    default: break;
    }
    assert(
        false); // "tried to compute the boundary of an tri mesh for an invalid simplex dimension"
    return false;
}

bool TriMesh::is_boundary_edge(const Tuple& tuple) const
{
    assert(is_valid(tuple));
    return m_ff_accessor->const_vector_attribute<3>(tuple)(tuple.local_eid()) < 0;
}

bool TriMesh::is_boundary_vertex(const Tuple& vertex) const
{
    // go through all edges and check if they are boundary
    // const simplex::SimplexCollection neigh = simplex::open_star(*this, Simplex::vertex(vertex));
    // for (const Simplex& s : neigh.get_edges()) {
    //    if (is_boundary(s.tuple())) {
    //        return true;
    //    }
    //}

    Tuple t = vertex;
    do {
        if (is_boundary_edge(t)) {
            return true;
        }
        t = switch_edge(switch_face(t));
    } while (t != vertex);

    return false;
}

Tuple TriMesh::switch_tuple(const Tuple& tuple, PrimitiveType type) const
{
    assert(is_valid(tuple));
    bool ccw = is_ccw(tuple);

    switch (type) {
    case PrimitiveType::Triangle: {
        const int64_t gvid = id(tuple, PrimitiveType::Vertex);
        const int64_t geid = id(tuple, PrimitiveType::Edge);
        const int64_t gfid = id(tuple, PrimitiveType::Triangle);

        auto ff = m_ff_accessor->const_vector_attribute<3>(tuple);

        int64_t gcid_new = ff(tuple.local_eid());
        int64_t lvid_new = -1, leid_new = -1;

        auto fv = m_fv_accessor->index_access().const_vector_attribute<3>(gcid_new);

        auto fe = m_fe_accessor->index_access().const_vector_attribute<3>(gcid_new);

        if (gfid == gcid_new) {
            // this supports 0,1,0 triangles not 0,0,0 triangles
            int64_t oleid = tuple.local_eid();
            int64_t olvid = tuple.local_vid();
            for (int64_t i = 0; i < 3; ++i) {
                if (i != oleid && fe(i) == geid) {
                    leid_new = i;
                }
            }
            // if the old vertex is no "opposite of the old or new edges
            // then they share the vertex
            //   a
            // 0/  \1 <--  0 and c share local ids, 1 and b share local ids
            // /____\.
            // b     c
            if (oleid != olvid && leid_new != olvid) {
                lvid_new = olvid;
            } else {
                for (int64_t i = 0; i < 3; ++i) {
                    if (i != olvid && fv(i) == gvid) {
                        lvid_new = i;
                    }
                }
            }
        } else {
            for (int64_t i = 0; i < 3; ++i) {
                if (fe(i) == geid) {
                    leid_new = i;
                }
                if (fv(i) == gvid) {
                    lvid_new = i;
                }
            }
        }
        assert(lvid_new != -1);
        assert(leid_new != -1);

        const Tuple res(lvid_new, leid_new, tuple.local_fid(), gcid_new);
        assert(is_valid(res));
        return res;
    }
    case PrimitiveType::Vertex:
    case PrimitiveType::Edge: return autogen::tri_mesh::local_switch_tuple(tuple, type);
    case PrimitiveType::Tetrahedron:
    default: {
        assert(false);
        return autogen::tri_mesh::local_switch_tuple(tuple, type);
    }
    }
}

bool TriMesh::is_ccw(const Tuple& tuple) const
{
    assert(is_valid(tuple));
    return autogen::tri_mesh::is_ccw(tuple);
}

void TriMesh::initialize(
    Eigen::Ref<const RowVectors3l> FV,
    Eigen::Ref<const RowVectors3l> FE,
    Eigen::Ref<const RowVectors3l> FF,
    Eigen::Ref<const VectorXl> VF,
    Eigen::Ref<const VectorXl> EF)
{
    // reserve memory for attributes


    std::vector<int64_t> cap{
        static_cast<int64_t>(VF.rows()),
        static_cast<int64_t>(EF.rows()),
        static_cast<int64_t>(FF.rows())};

    set_capacities(cap);


    // get Accessors for topology
    attribute::Accessor<int64_t> fv_accessor = create_accessor<int64_t>(m_fv_handle);
    attribute::Accessor<int64_t> fe_accessor = create_accessor<int64_t>(m_fe_handle);
    attribute::Accessor<int64_t> ff_accessor = create_accessor<int64_t>(m_ff_handle);
    attribute::Accessor<int64_t> vf_accessor = create_accessor<int64_t>(m_vf_handle);
    attribute::Accessor<int64_t> ef_accessor = create_accessor<int64_t>(m_ef_handle);

    attribute::FlagAccessor<TriMesh> v_flag_accessor = get_flag_accessor(PrimitiveType::Vertex);
    attribute::FlagAccessor<TriMesh> e_flag_accessor = get_flag_accessor(PrimitiveType::Edge);
    attribute::FlagAccessor<TriMesh> f_flag_accessor = get_flag_accessor(PrimitiveType::Triangle);

    // iterate over the matrices and fill attributes
    for (int64_t i = 0; i < capacity(PrimitiveType::Triangle); ++i) {
        fv_accessor.index_access().vector_attribute<3>(i) = FV.row(i).transpose();
        fe_accessor.index_access().vector_attribute<3>(i) = FE.row(i).transpose();
        ff_accessor.index_access().vector_attribute<3>(i) = FF.row(i).transpose();

        f_flag_accessor.index_access().activate(i);
    }
    // m_vf
    for (int64_t i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        auto& vf = vf_accessor.index_access().scalar_attribute(i);
        vf = VF(i);
        v_flag_accessor.index_access().activate(i);
    }
    // m_ef
    for (int64_t i = 0; i < capacity(PrimitiveType::Edge); ++i) {
        auto& ef = ef_accessor.index_access().scalar_attribute(i);
        ef = EF(i);
        e_flag_accessor.index_access().activate(i);
    }
}

void TriMesh::initialize(Eigen::Ref<const RowVectors3l> F, bool is_free)
{
    this->m_is_free = is_free;
    auto [FE, FF, VF, EF] = trimesh_topology_initialization(F);
    if (is_free) {
        FF.setConstant(-1);
    }
    initialize(F, FE, FF, VF, EF);
}
void TriMesh::initialize_free(int64_t count)
{
    // 0 1 2
    // 3 4 5
    RowVectors3l S(count, 3);
    std::iota(S.data(), S.data() + S.size(), int64_t(0));
    initialize(S, true);
}

Tuple TriMesh::tuple_from_global_ids(int64_t fid, int64_t eid, int64_t vid) const
{
    auto fv = m_fv_accessor->index_access().const_vector_attribute<3>(fid);
    auto fe = m_fe_accessor->index_access().const_vector_attribute<3>(fid);


    int64_t lvid = -1;
    int64_t leid = -1;

    for (int j = 0; j < 3; ++j) {
        if (fv(j) == vid) {
            lvid = j;
        }
        if (fe(j) == eid) {
            leid = j;
        }
    }
    assert(lvid != -1);
    assert(leid != -1);

    return Tuple(lvid, leid, -1, fid);
}

Tuple TriMesh::tuple_from_id(const PrimitiveType type, const int64_t gid) const
{
    switch (type) {
    case PrimitiveType::Vertex: {
        return vertex_tuple_from_id(gid);
    }
    case PrimitiveType::Edge: {
        return edge_tuple_from_id(gid);
    }
    case PrimitiveType::Triangle: {
        return face_tuple_from_id(gid);
    }
    case PrimitiveType::Tetrahedron: {
        throw std::runtime_error("no tet tuple supported for trimesh");
        break;
    }
    default: assert(false); // "Invalid primitive type"
    }
    return Tuple();
}

Tuple TriMesh::vertex_tuple_from_id(int64_t id) const
{
    auto f = m_vf_accessor->index_access().const_scalar_attribute(id);
    auto fv = m_fv_accessor->index_access().const_vector_attribute<3>(f);
    for (int64_t i = 0; i < 3; ++i) {
        if (fv(i) == id) {
            return autogen::tri_mesh::get_tuple_from_simplex_local_vertex_id(i, f);
        }
    }
    assert(false); // "vertex_tuple_from_id failed"

    return Tuple();
}

Tuple TriMesh::edge_tuple_from_id(int64_t id) const
{
    auto f = m_ef_accessor->index_access().const_scalar_attribute(id);
    auto fe = m_fe_accessor->index_access().const_vector_attribute<3>(f);
    for (int64_t i = 0; i < 3; ++i) {
        if (fe(i) == id) {
            return autogen::tri_mesh::get_tuple_from_simplex_local_edge_id(i, f);
        }
    }
    assert(false); // "edge_tuple_from_id failed"

    return Tuple();
}

Tuple TriMesh::face_tuple_from_id(int64_t id) const
{
    Tuple f_tuple = Tuple(
        autogen::tri_mesh::auto_2d_table_complete_vertex[0][0],
        autogen::tri_mesh::auto_2d_table_complete_vertex[0][1],
        -1,
        id);
    assert(is_ccw(f_tuple));
    assert(is_valid(f_tuple));
    return f_tuple;
}

bool TriMesh::is_valid(const Tuple& tuple) const
{
    if (!Mesh::is_valid(tuple)) {
        logger().trace("Base Mesh type reported invalidity");
        return false;
    }
    const bool is_connectivity_valid = tuple.local_vid() >= 0 && tuple.local_eid() >= 0 &&
                                       tuple.global_cid() >= 0 &&
                                       autogen::tri_mesh::tuple_is_valid_for_ccw(tuple);

    if (!is_connectivity_valid) {
#if !defined(NDEBUG)
        logger().trace(
            "tuple.local_vid()={} >= 0 && tuple.local_eid()={} >= 0 &&"
            " tuple.global_cid()={} >= 0 &&"
            " autogen::tri_mesh::tuple_is_valid_for_ccw(tuple)={}",
            tuple.local_vid(),
            tuple.local_eid(),
            tuple.global_cid(),
            autogen::tri_mesh::tuple_is_valid_for_ccw(tuple));
        assert(tuple.local_vid() >= 0);
        assert(tuple.local_eid() >= 0);
        assert(tuple.global_cid() >= 0);
        assert(autogen::tri_mesh::tuple_is_valid_for_ccw(tuple));
#endif
        return false;
    }
    return true;
}

bool TriMesh::is_connectivity_valid() const
{
    // get Accessors for topology
    const attribute::Accessor<int64_t> fv_accessor = create_const_accessor<int64_t>(m_fv_handle);
    const attribute::Accessor<int64_t> fe_accessor = create_const_accessor<int64_t>(m_fe_handle);
    const attribute::Accessor<int64_t> ff_accessor = create_const_accessor<int64_t>(m_ff_handle);
    const attribute::Accessor<int64_t> vf_accessor = create_const_accessor<int64_t>(m_vf_handle);
    const attribute::Accessor<int64_t> ef_accessor = create_const_accessor<int64_t>(m_ef_handle);
    const attribute::FlagAccessor<TriMesh> v_flag_accessor =
        get_flag_accessor(PrimitiveType::Vertex);
    const attribute::FlagAccessor<TriMesh> e_flag_accessor = get_flag_accessor(PrimitiveType::Edge);
    const attribute::FlagAccessor<TriMesh> f_flag_accessor =
        get_flag_accessor(PrimitiveType::Triangle);


    for (int64_t i = 0; i < capacity(PrimitiveType::Triangle); ++i) {
        if (!f_flag_accessor.index_access().is_active(i)) {
            continue;
        }
        auto fe = fe_accessor.index_access().const_vector_attribute<3>(i);
        auto fv = fv_accessor.index_access().const_vector_attribute<3>(i);

        bool bad_face = false;

        for (int64_t j = 0; j < 3; ++j) {
            int64_t ei = fe(j);
            int64_t vi = fv(j);
            if (!e_flag_accessor.index_access().is_active(ei)) {
                wmtk::logger().error(
                    "Face {} refers to edge {} at local index {} which was deleted",
                    i,
                    ei,
                    j);
                bad_face = true;
            }
            if (!v_flag_accessor.index_access().is_active(vi)) {
                wmtk::logger().error(
                    "Face {} refers to vertex{} at local index {} which was deleted",
                    i,
                    vi,
                    j);
                bad_face = true;
            }
        }
        if (bad_face) {
            return false;
        }
    }
    // EF and FE
    for (int64_t i = 0; i < capacity(PrimitiveType::Edge); ++i) {
        if (!e_flag_accessor.index_access().is_active(i)) {
            continue;
        }
        int cnt = 0;
        long ef_val = ef_accessor.index_access().const_scalar_attribute(i);

        auto fe_val = fe_accessor.index_access().const_vector_attribute<3>(ef_val);
        for (int64_t j = 0; j < 3; ++j) {
            if (fe_val(j) == i) {
                cnt++;
            }
        }
        if (cnt == 0) {
            wmtk::logger().error(
                "EF[{0}] {1} and FE:[EF[{0}]] = {2} are not "
                "compatible ",
                i,
                ef_val,
                fmt::join(fe_val, ","));

            // std::cout << "EF and FE not compatible" << std::endl;
            return false;
        }
    }

    // VF and FV
    for (int64_t i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        const int64_t vf = vf_accessor.index_access().const_scalar_attribute(i);
        if (!v_flag_accessor.index_access().is_active(i)) {
            continue;
        }
        int cnt = 0;

        auto fv = fv_accessor.index_access().const_vector_attribute<3>(vf);
        for (int64_t j = 0; j < 3; ++j) {
            if (fv(j) == i) {
                cnt++;
            }
        }
        if (cnt == 0) {
            wmtk::logger().error(
                "VF and FV not compatible, could not find VF[{}] = {} "
                "in FV[{}] = [{}]",
                i,
                vf,
                vf,
                fmt::join(fv, ","));
            return false;
        }
    }

    // FE and EF
    for (int64_t i = 0; i < capacity(PrimitiveType::Triangle); ++i) {
        if (!f_flag_accessor.index_access().is_active(i)) {
            continue;
        }
        auto fe = fe_accessor.index_access().const_vector_attribute<3>(i);
        auto ff = ff_accessor.index_access().const_vector_attribute<3>(i);

        for (int64_t j = 0; j < 3; ++j) {
            int neighbor_fid = ff(j);
            const bool is_boundary = neighbor_fid == -1;
            if (is_boundary) {
                auto ef = ef_accessor.index_access().const_scalar_attribute(fe(j));
                if (ef != i) {
                    wmtk::logger().error(
                        "Even though local edge {} of face {} is "
                        "boundary (global eid is {}), "
                        "ef[{}] = {} != {}",
                        j,
                        i,
                        fe(j),
                        fe(j),
                        ef,
                        i);
                    return false;
                }
            } else {
                if (neighbor_fid == i) {
                    logger().error(
                        "Connectivity check cannot work when mapping a "
                        "face to itself (face {})",
                        i);
                    assert(false);
                    continue;
                }
                auto neighbor_ff =
                    ff_accessor.index_access().const_vector_attribute<3>(neighbor_fid);

                if ((neighbor_ff.array() == i).any()) {
                    auto neighbor_fe =
                        fe_accessor.index_access().const_vector_attribute<3>(neighbor_fid);

                    int edge_shared_count = 0;
                    for (int local_neighbor_eid = 0; local_neighbor_eid < 3; ++local_neighbor_eid) {
                        // find some edge which is shared
                        if (neighbor_ff(local_neighbor_eid) == i) {
                            if (fe(j) == neighbor_fe(local_neighbor_eid)) {
                                edge_shared_count++;
                            }
                        }
                    }
                    if (edge_shared_count != 1) {
                        wmtk::logger().error(
                            "face {} with fe={} neighbor fe[{}] = {} "
                            "was unable to find itself "
                            "uniquely (found {})",
                            i,
                            fmt::join(fe, ","),
                            neighbor_fid,
                            fmt::join(neighbor_fe, ","),
                            edge_shared_count);
                        return false;
                    }
                } else {
                    wmtk::logger().error(
                        "face {} with ff={} neighbor ff[{}] = {} was "
                        "unable to find itself",
                        i,
                        fmt::join(ff, ","),
                        neighbor_fid,
                        fmt::join(neighbor_ff, ","));
                    return false;
                }
            }
        }
    }

    return true;
}

Tuple TriMesh::with_different_cid(const Tuple& t, int64_t cid)
{
    Tuple r(t.local_vid(), t.local_eid(), t.local_fid(), cid);
    return r;
}

std::vector<std::vector<TypedAttributeHandle<int64_t>>> TriMesh::connectivity_attributes() const
{
    std::vector<std::vector<TypedAttributeHandle<int64_t>>> handles(3);

    handles[2].push_back(m_vf_handle);
    handles[2].push_back(m_ef_handle);
    handles[2].push_back(m_ff_handle);

    handles[1].push_back(m_fe_handle);

    handles[0].push_back(m_fv_handle);

    return handles;
}

std::vector<Tuple> TriMesh::orient_vertices(const Tuple& tuple) const
{
    int64_t cid = tuple.global_cid();
    return {Tuple(0, 2, -1, cid), Tuple(1, 0, -1, cid), Tuple(2, 1, -1, cid)};
}


} // namespace wmtk
