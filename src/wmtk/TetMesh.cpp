#include "TetMesh.hpp"


#include <wmtk/utils/tetmesh_topology_initialization.h>
#include <numeric>
#include <wmtk/autogen/tet_mesh/is_ccw.hpp>
#include <wmtk/autogen/tet_mesh/local_switch_tuple.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/open_star.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk {

using namespace autogen;

TetMesh::TetMesh()
    : MeshCRTP<TetMesh>(3)
    , m_vt_handle(register_attribute_typed<int64_t>("m_vt", PrimitiveType::Vertex, 1, false, -1))
    , m_et_handle(register_attribute_typed<int64_t>("m_et", PrimitiveType::Edge, 1, false, -1))
    , m_ft_handle(register_attribute_typed<int64_t>("m_ft", PrimitiveType::Triangle, 1, false, -1))
    , m_tv_handle(
          register_attribute_typed<int64_t>("m_tv", PrimitiveType::Tetrahedron, 4, false, -1))
    , m_te_handle(
          register_attribute_typed<int64_t>("m_te", PrimitiveType::Tetrahedron, 6, false, -1))
    , m_tf_handle(
          register_attribute_typed<int64_t>("m_tf", PrimitiveType::Tetrahedron, 4, false, -1))
    , m_tt_handle(
          register_attribute_typed<int64_t>("m_tt", PrimitiveType::Tetrahedron, 4, false, -1))
{
    make_cached_accessors();
}


TetMesh::TetMesh(TetMesh&& o)
    : MeshCRTP<TetMesh>(std::move(o))
{
    m_vt_handle = o.m_vt_handle;
    m_et_handle = o.m_et_handle;
    m_ft_handle = o.m_ft_handle;
    m_tv_handle = o.m_tv_handle;
    m_te_handle = o.m_te_handle;
    m_tf_handle = o.m_tf_handle;
    m_tt_handle = o.m_tt_handle;

    make_cached_accessors();
}
TetMesh& TetMesh::operator=(TetMesh&& o)
{
    Mesh::operator=(std::move(o));
    m_vt_handle = o.m_vt_handle;
    m_et_handle = o.m_et_handle;
    m_ft_handle = o.m_ft_handle;
    m_tv_handle = o.m_tv_handle;
    m_te_handle = o.m_te_handle;
    m_tf_handle = o.m_tf_handle;
    m_tt_handle = o.m_tt_handle;

    make_cached_accessors();
    return *this;
}


void TetMesh::make_cached_accessors()
{
    m_vt_accessor = std::make_unique<attribute::Accessor<int64_t, TetMesh>>(*this, m_vt_handle);
    m_et_accessor = std::make_unique<attribute::Accessor<int64_t, TetMesh>>(*this, m_et_handle);
    m_ft_accessor = std::make_unique<attribute::Accessor<int64_t, TetMesh>>(*this, m_ft_handle);

    m_tv_accessor = std::make_unique<attribute::Accessor<int64_t, TetMesh>>(*this, m_tv_handle);
    m_te_accessor = std::make_unique<attribute::Accessor<int64_t, TetMesh>>(*this, m_te_handle);
    m_tf_accessor = std::make_unique<attribute::Accessor<int64_t, TetMesh>>(*this, m_tf_handle);
    m_tt_accessor = std::make_unique<attribute::Accessor<int64_t, TetMesh>>(*this, m_tt_handle);
}


void TetMesh::initialize(
    Eigen::Ref<const RowVectors4l> TV,
    Eigen::Ref<const RowVectors6l> TE,
    Eigen::Ref<const RowVectors4l> TF,
    Eigen::Ref<const RowVectors4l> TT,
    Eigen::Ref<const VectorXl> VT,
    Eigen::Ref<const VectorXl> ET,
    Eigen::Ref<const VectorXl> FT)

{
    // reserve memory for attributes

    std::vector<int64_t> cap{
        static_cast<int64_t>(VT.rows()),
        static_cast<int64_t>(ET.rows()),
        static_cast<int64_t>(FT.rows()),
        static_cast<int64_t>(TT.rows())};
    set_capacities(cap);

    // get Accessors for topology
    auto vt_accessor = create_accessor<int64_t>(m_vt_handle);
    auto et_accessor = create_accessor<int64_t>(m_et_handle);
    auto ft_accessor = create_accessor<int64_t>(m_ft_handle);
    auto tv_accessor = create_accessor<int64_t>(m_tv_handle);
    auto te_accessor = create_accessor<int64_t>(m_te_handle);
    auto tf_accessor = create_accessor<int64_t>(m_tf_handle);
    auto tt_accessor = create_accessor<int64_t>(m_tt_handle);
    attribute::Accessor<char> v_flag_accessor = get_flag_accessor(PrimitiveType::Vertex);
    attribute::Accessor<char> e_flag_accessor = get_flag_accessor(PrimitiveType::Edge);
    attribute::Accessor<char> f_flag_accessor = get_flag_accessor(PrimitiveType::Triangle);
    attribute::Accessor<char> t_flag_accessor = get_flag_accessor(PrimitiveType::Tetrahedron);

    // iterate over the matrices and fill attributes
    for (int64_t i = 0; i < capacity(PrimitiveType::Tetrahedron); ++i) {
        tv_accessor.index_access().vector_attribute<4>(i) = TV.row(i).transpose();
        te_accessor.index_access().vector_attribute<6>(i) = TE.row(i).transpose();
        tf_accessor.index_access().vector_attribute<4>(i) = TF.row(i).transpose();
        tt_accessor.index_access().vector_attribute<4>(i) = TT.row(i).transpose();
        t_flag_accessor.index_access().scalar_attribute(i) |= 0x1;
    }
    // m_vt
    for (int64_t i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        vt_accessor.index_access().scalar_attribute(i) = VT(i);
        v_flag_accessor.index_access().scalar_attribute(i) |= 0x1;
    }
    // m_et
    for (int64_t i = 0; i < capacity(PrimitiveType::Edge); ++i) {
        et_accessor.index_access().scalar_attribute(i) = ET(i);
        e_flag_accessor.index_access().scalar_attribute(i) |= 0x1;
    }
    // m_ft
    for (int64_t i = 0; i < capacity(PrimitiveType::Triangle); ++i) {
        ft_accessor.index_access().scalar_attribute(i) = FT(i);
        f_flag_accessor.index_access().scalar_attribute(i) |= 0x1;
    }
}


void TetMesh::initialize(Eigen::Ref<const RowVectors4l> T, bool is_free)
{
    this->m_is_free = is_free;
    auto [TE, TF, TT, VT, ET, FT] = tetmesh_topology_initialization(T);
    if (is_free) {
        assert(false); // TODO: tet operations currently dont work yet so this optino isn't allowed
        TT.setConstant(-1);
    }
    initialize(T, TE, TF, TT, VT, ET, FT);
}
void TetMesh::initialize_free(int64_t count)
{
    RowVectors4l S(count, 4);
    std::iota(S.data(), S.data() + S.size(), int64_t(0));
    initialize(S, true);
}

Tuple TetMesh::vertex_tuple_from_id(int64_t id) const
{
    int64_t t = m_vt_accessor->index_access().const_scalar_attribute(id);
    auto tv = m_tv_accessor->index_access().const_vector_attribute<4>(t);
    int64_t lvid = -1;

    for (int64_t i = 0; i < 4; ++i) {
        if (tv(i) == id) {
            lvid = i;
            break;
        }
    }

    const auto [nlvid, leid, lfid] = autogen::tet_mesh::auto_3d_table_complete_vertex[lvid];
    assert(lvid == nlvid);

    if (lvid < 0 || leid < 0 || lfid < 0) throw std::runtime_error("vertex_tuple_from_id failed");

#if defined(WMTK_ENABLE_HASH_UPDATE)
    const attribute::Accessor<int64_t> hash_accessor = get_const_cell_hash_accessor();

    Tuple v_tuple = Tuple(lvid, leid, lfid, t, get_cell_hash(t, hash_accessor));
#else
    Tuple v_tuple = Tuple(lvid, leid, lfid, t);
#endif
    assert(is_ccw(v_tuple));
    assert(is_valid(v_tuple));
    return v_tuple;
}

Tuple TetMesh::edge_tuple_from_id(int64_t id) const
{
    int64_t t = m_et_accessor->index_access().const_scalar_attribute(id);
    auto te = m_te_accessor->index_access().const_vector_attribute<6>(t);

    int64_t leid = -1;

    for (int64_t i = 0; i < 6; ++i) {
        if (te(i) == id) {
            leid = i;
            break;
        }
    }
    const auto [lvid, nleid, lfid] = autogen::tet_mesh::auto_3d_table_complete_edge[leid];
    assert(leid == nleid);


    if (lvid < 0 || leid < 0 || lfid < 0) throw std::runtime_error("edge_tuple_from_id failed");

#if defined(WMTK_ENABLE_HASH_UPDATE)
    const attribute::Accessor<int64_t> hash_accessor = get_const_cell_hash_accessor();

    Tuple e_tuple = Tuple(lvid, leid, lfid, t, get_cell_hash(t, hash_accessor));
#else
    Tuple e_tuple = Tuple(lvid, leid, lfid, t);
#endif
    assert(is_ccw(e_tuple));
    assert(is_valid(e_tuple));
    return e_tuple;
}

Tuple TetMesh::face_tuple_from_id(int64_t id) const
{
    int64_t t = m_ft_accessor->index_access().const_scalar_attribute(id);
    auto tf = m_tf_accessor->index_access().const_vector_attribute<4>(t);

    int64_t lfid = -1;

    for (int64_t i = 0; i < 4; ++i) {
        if (tf(i) == id) {
            lfid = i;
            break;
        }
    }

    const auto [lvid, leid, nlfid] = autogen::tet_mesh::auto_3d_table_complete_face[lfid];
    assert(lfid == nlfid);

    if (lvid < 0 || leid < 0 || lfid < 0) throw std::runtime_error("face_tuple_from_id failed");

#if defined(WMTK_ENABLE_HASH_UPDATE)
    const attribute::Accessor<int64_t> hash_accessor = get_const_cell_hash_accessor();

    Tuple f_tuple = Tuple(lvid, leid, lfid, t, get_cell_hash(t, hash_accessor));
#else
    Tuple f_tuple = Tuple(lvid, leid, lfid, t);
#endif
    assert(is_ccw(f_tuple));
    assert(is_valid(f_tuple));
    return f_tuple;
}

Tuple TetMesh::tet_tuple_from_id(int64_t id) const
{
    const int64_t lvid = 0;
    const auto [nlvid, leid, lfid] = autogen::tet_mesh::auto_3d_table_complete_vertex[lvid];
    assert(lvid == nlvid);

#if defined(WMTK_ENABLE_HASH_UPDATE)
    const attribute::Accessor<int64_t> hash_accessor = get_const_cell_hash_accessor();

    Tuple t_tuple = Tuple(lvid, leid, lfid, id, get_cell_hash(id, hash_accessor));
#else
    Tuple t_tuple = Tuple(lvid, leid, lfid, id);
#endif
    assert(is_ccw(t_tuple));
    assert(is_valid(t_tuple));
    return t_tuple;
}

Tuple TetMesh::tuple_from_id(const PrimitiveType type, const int64_t gid) const
{
    switch (type) {
    case PrimitiveType::Vertex: {
        return vertex_tuple_from_id(gid);
        break;
    }
    case PrimitiveType::Edge: {
        return edge_tuple_from_id(gid);
        break;
    }
    case PrimitiveType::Triangle: {
        return face_tuple_from_id(gid);
        break;
    }
    case PrimitiveType::Tetrahedron: {
        return tet_tuple_from_id(gid);
        break;
    }
    default: assert(false); // "Invalid primitive type"
    }

    return Tuple();
}


Tuple TetMesh::switch_tuple(const Tuple& tuple, PrimitiveType type) const
{
    assert(is_valid(tuple));
    switch (type) {
    // bool ccw = is_ccw(tuple);
    case PrimitiveType::Tetrahedron: {
        assert(!is_boundary_face(tuple));
        // need test
        const int64_t gvid = id(tuple, PrimitiveType::Vertex);
        const int64_t geid = id(tuple, PrimitiveType::Edge);
        const int64_t gfid = id(tuple, PrimitiveType::Triangle);

        auto tt = m_tt_accessor->const_vector_attribute<4>(tuple);

        int64_t gcid_new = tt(tuple.m_local_fid);

        /*handle exception here*/
        assert(gcid_new != -1);
        // check if is_boundary allows removing this exception in 3d cases
        // if (gcid_new == -1) {
        //     return Tuple(-1, -1, -1, -1, -1);
        // }
        /*handle exception end*/

        int64_t lvid_new = -1, leid_new = -1, lfid_new = -1;

        auto tv = m_tv_accessor->index_access().const_vector_attribute<4>(gcid_new);

        auto te = m_te_accessor->index_access().const_vector_attribute<6>(gcid_new);

        auto tf = m_tf_accessor->index_access().const_vector_attribute<4>(gcid_new);

        for (int64_t i = 0; i < 4; ++i) {
            if (tv(i) == gvid) {
                lvid_new = i;
            }
            if (tf(i) == gfid) {
                lfid_new = i;
            }
        }

        for (int64_t i = 0; i < 6; ++i) {
            if (te(i) == geid) {
                leid_new = i;
                break; // check if the break is correct
            }
        }


        assert(lvid_new != -1);
        assert(leid_new != -1);
        assert(lfid_new != -1);

#if defined(WMTK_ENABLE_HASH_UPDATE)
        const Tuple res(lvid_new, leid_new, lfid_new, gcid_new, get_cell_hash_slow(gcid_new));
#else
        const Tuple res(lvid_new, leid_new, lfid_new, gcid_new);
#endif
        assert(is_valid(res));
        return res;
    }
    case PrimitiveType::Vertex:
    case PrimitiveType::Edge:
    case PrimitiveType::Triangle:
    default: return autogen::tet_mesh::local_switch_tuple(tuple, type);
    }
}

bool TetMesh::is_ccw(const Tuple& tuple) const
{
    assert(is_valid(tuple));
    return autogen::tet_mesh::is_ccw(tuple);
}

bool TetMesh::is_valid(const Tuple& tuple) const
{
    if (!Mesh::is_valid(tuple)) {
        return false;
    }
    const bool is_connectivity_valid = tuple.m_local_vid >= 0 && tuple.m_local_eid >= 0 &&
                                       tuple.m_local_fid >= 0 && tuple.m_global_cid >= 0 &&
                                       autogen::tet_mesh::tuple_is_valid_for_ccw(tuple);

    if (!is_connectivity_valid) {
        return false;
    }

    return true;
}

bool TetMesh::is_boundary(PrimitiveType pt, const Tuple& tuple) const
{
    switch (pt) {
    case PrimitiveType::Vertex: return is_boundary_vertex(tuple);
    case PrimitiveType::Edge: return is_boundary_edge(tuple);
    case PrimitiveType::Triangle: return is_boundary_face(tuple);
    case PrimitiveType::Tetrahedron:
    default: break;
    }
    assert(
        false); // "tried to compute the boundary of an tet mesh for an invalid simplex dimension"
    return false;
}


bool TetMesh::is_boundary_face(const Tuple& tuple) const
{
    const attribute::Accessor<int64_t> tt_accessor = create_const_accessor<int64_t>(m_tt_handle);
    return tt_accessor.const_vector_attribute<4>(tuple)(tuple.m_local_fid) < 0;
}

bool TetMesh::is_boundary_edge(const Tuple& edge) const
{
    for (const Tuple& f : simplex::cofaces_single_dimension_tuples(
             *this,
             simplex::Simplex::edge(*this, edge),
             PrimitiveType::Triangle)) {
        if (is_boundary_face(f)) {
            return true;
        }
    }
    return false;
}
bool TetMesh::is_boundary_vertex(const Tuple& vertex) const
{
    // go through all faces and check if they are boundary
    const simplex::SimplexCollection neigh =
        wmtk::simplex::open_star(*this, simplex::Simplex::vertex(*this, vertex));
    for (const simplex::Simplex& s : neigh.simplex_vector(PrimitiveType::Triangle)) {
        if (is_boundary(s)) {
            return true;
        }
    }

    return false;
}

bool TetMesh::is_connectivity_valid() const
{
    // get Accessors for topology
    const attribute::Accessor<int64_t> tv_accessor = create_const_accessor<int64_t>(m_tv_handle);
    const attribute::Accessor<int64_t> te_accessor = create_const_accessor<int64_t>(m_te_handle);
    const attribute::Accessor<int64_t> tf_accessor = create_const_accessor<int64_t>(m_tf_handle);
    const attribute::Accessor<int64_t> tt_accessor = create_const_accessor<int64_t>(m_tt_handle);
    const attribute::Accessor<int64_t> vt_accessor = create_const_accessor<int64_t>(m_vt_handle);
    const attribute::Accessor<int64_t> et_accessor = create_const_accessor<int64_t>(m_et_handle);
    const attribute::Accessor<int64_t> ft_accessor = create_const_accessor<int64_t>(m_ft_handle);
    const attribute::Accessor<char> v_flag_accessor = get_flag_accessor(PrimitiveType::Vertex);
    const attribute::Accessor<char> e_flag_accessor = get_flag_accessor(PrimitiveType::Edge);
    const attribute::Accessor<char> f_flag_accessor = get_flag_accessor(PrimitiveType::Triangle);
    const attribute::Accessor<char> t_flag_accessor = get_flag_accessor(PrimitiveType::Tetrahedron);

    // VT and TV
    for (int64_t i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        if (v_flag_accessor.index_access().const_scalar_attribute(i) == 0) {
            wmtk::logger().debug("Vertex {} is deleted", i);
            continue;
        }
        int cnt = 0;
        for (int j = 0; j < 4; ++j) {
            if (tv_accessor.index_access().const_vector_attribute<4>(
                    vt_accessor.index_access().const_scalar_attribute(i))[j] == i) {
                cnt++;
            }
        }
        if (cnt != 1) {
            wmtk::logger().info("fail VT and TV");
            return false;
        }
    }

    // ET and TE
    for (int64_t i = 0; i < capacity(PrimitiveType::Edge); ++i) {
        if (e_flag_accessor.index_access().const_scalar_attribute(i) == 0) {
            wmtk::logger().debug("Edge {} is deleted", i);
            continue;
        }
        int cnt = 0;
        for (int j = 0; j < 6; ++j) {
            if (te_accessor.index_access().const_vector_attribute<6>(
                    et_accessor.index_access().const_scalar_attribute(i))[j] == i) {
                cnt++;
            }
        }
        if (cnt != 1) {
            wmtk::logger().info("fail ET and TE");
            return false;
        }
    }

    // FT and TF
    for (int64_t i = 0; i < capacity(PrimitiveType::Triangle); ++i) {
        if (f_flag_accessor.index_access().const_scalar_attribute(i) == 0) {
            wmtk::logger().debug("Face {} is deleted", i);
            continue;
        }
        int cnt = 0;
        for (int j = 0; j < 4; ++j) {
            if (tf_accessor.index_access().const_vector_attribute<4>(
                    ft_accessor.index_access().const_scalar_attribute(i))[j] == i) {
                cnt++;
            }
        }
        if (cnt != 1) {
            wmtk::logger().info("fail FT and TF");
            return false;
        }
    }

    // TF and TT
    for (int64_t i = 0; i < capacity(PrimitiveType::Tetrahedron); ++i) {
        if (t_flag_accessor.index_access().const_scalar_attribute(i) == 0) {
            wmtk::logger().debug("Tet {} is deleted", i);
            continue;
        }

        for (int j = 0; j < 4; ++j) {
            int64_t nb = tt_accessor.index_access().const_vector_attribute<4>(i)(j);
            if (nb == -1) {
                if (ft_accessor.index_access().const_scalar_attribute(
                        tf_accessor.index_access().const_vector_attribute<4>(i)(j)) != i) {
                    wmtk::logger().info("fail TF and TT 1");
                    return false;
                }
                continue;
            }

            int cnt = 0;
            int id_in_nb;
            for (int k = 0; k < 4; ++k) {
                if (tt_accessor.index_access().const_vector_attribute<4>(nb)(k) == i) {
                    cnt++;
                    id_in_nb = k;
                }
            }
            if (cnt != 1) {
                wmtk::logger().info("fail TF and TT 2");
                return false;
            }

            if (tf_accessor.index_access().const_vector_attribute<4>(i)(j) !=
                tf_accessor.index_access().const_vector_attribute<4>(nb)(id_in_nb)) {
                wmtk::logger().info("fail TF and TT 3");
                return false;
            }
        }
    }

    return true;
}

std::vector<std::vector<TypedAttributeHandle<int64_t>>> TetMesh::connectivity_attributes() const
{
    std::vector<std::vector<TypedAttributeHandle<int64_t>>> handles(4);

    handles[0].push_back(m_tv_handle);
    handles[1].push_back(m_te_handle);
    handles[2].push_back(m_tf_handle);

    handles[3].push_back(m_tt_handle);
    handles[3].push_back(m_vt_handle);
    handles[3].push_back(m_et_handle);
    handles[3].push_back(m_ft_handle);

    return handles;
}

Tuple TetMesh::tuple_from_global_ids(int64_t tid, int64_t fid, int64_t eid, int64_t vid) const
{
    auto tv = m_tv_accessor->index_access().const_vector_attribute<4>(tid);
    auto te = m_te_accessor->index_access().const_vector_attribute<6>(tid);
    auto tf = m_tf_accessor->index_access().const_vector_attribute<4>(tid);

    int64_t lvid = -1, leid = -1, lfid = -1;

    for (int j = 0; j < 4; ++j) {
        if (tv(j) == vid) {
            lvid = j;
        }
        if (tf(j) == fid) {
            lfid = j;
        }
    }

    for (int j = 0; j < 6; ++j) {
        if (te(j) == eid) {
            leid = j;
            break;
        }
    }

    assert(lvid != -1);
    assert(leid != -1);
    assert(lfid != -1);

#if defined(WMTK_ENABLE_HASH_UPDATE)
    return Tuple(lvid, leid, lfid, tid, get_cell_hash_slow(tid));
#else
    return Tuple(lvid, leid, lfid, tid);
#endif
}

std::vector<Tuple> TetMesh::orient_vertices(const Tuple& tuple) const
{
    int64_t cid = tuple.m_global_cid;
#if defined(WMTK_ENABLE_HASH_UPDATE)
    auto hash = get_cell_hash_slow(cid);

    return {
        Tuple(0, 0, 2, cid, hash),
        Tuple(1, 0, 3, cid, hash),
        Tuple(2, 1, 1, cid, hash),
        Tuple(3, 2, 2, cid, hash)};
#else
    return {Tuple(0, 0, 2, cid), Tuple(1, 0, 3, cid), Tuple(2, 1, 1, cid), Tuple(3, 2, 2, cid)};
#endif
}


} // namespace wmtk
