#include "TriMesh.hpp"

#include <wmtk/utils/trimesh_topology_initialization.h>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/autogen/autogenerated_2d_tables.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk {
TriMesh::TriMesh()
    : Mesh(2)
    , m_vf_handle(register_attribute<long>("m_vf", PrimitiveType::Vertex, 1))
    , m_ef_handle(register_attribute<long>("m_ef", PrimitiveType::Edge, 1))
    , m_fv_handle(register_attribute<long>("m_fv", PrimitiveType::Face, 3))
    , m_fe_handle(register_attribute<long>("m_fe", PrimitiveType::Face, 3))
    , m_ff_handle(register_attribute<long>("m_ff", PrimitiveType::Face, 3))
{}
TriMesh::TriMesh(const TriMesh& o) = default;
TriMesh::TriMesh(TriMesh&& o) = default;
TriMesh& TriMesh::operator=(const TriMesh& o) = default;
TriMesh& TriMesh::operator=(TriMesh&& o) = default;

Tuple TriMesh::split_edge(const Tuple& t)
{
    // record the deleted simplices topology attributes
    TriMesh::TriMeshOperationExecutor executor(*this, t);
    return executor.split_edge();
}

Tuple TriMesh::collapse_edge(const Tuple& t)
{
    TriMesh::TriMeshOperationExecutor executor(*this, t);
    return executor.collapse_edge();
}

long TriMesh::id(const Tuple& tuple, PrimitiveType type) const
{
    switch (type) {
    case PrimitiveType::Vertex: {
        ConstAccessor<long> fv_accessor = create_const_accessor<long>(m_fv_handle);
        auto fv = fv_accessor.vector_attribute(tuple);
        return fv(tuple.m_local_vid);
    }
    case PrimitiveType::Edge: {
        ConstAccessor<long> fe_accessor = create_const_accessor<long>(m_fe_handle);
        auto fe = fe_accessor.vector_attribute(tuple);
        return fe(tuple.m_local_eid);
    }
    case PrimitiveType::Face: {
        return tuple.m_global_cid;
    }
    case PrimitiveType::Tetrahedron:
    default: throw std::runtime_error("Tuple id: Invalid primitive type");
    }
}

bool TriMesh::is_boundary(const Tuple& tuple) const
{
    assert(is_valid(tuple));
    ConstAccessor<long> ff_accessor = create_const_accessor<long>(m_ff_handle);
    return ff_accessor.vector_attribute(tuple)(tuple.m_local_eid) < 0;
}

bool TriMesh::is_boundary_vertex(const Tuple& vertex) const
{
    // go through all edges and check if they are boundary
    const SimplicialComplex neigh = SimplicialComplex::open_star(*this, Simplex::vertex(vertex));
    for (const Simplex& s : neigh.get_edges()) {
        if (is_boundary(s.tuple())) {
            return true;
        }
    }

    return false;
}

Tuple TriMesh::switch_tuple(const Tuple& tuple, PrimitiveType type) const
{
    assert(is_valid(tuple));
    bool ccw = is_ccw(tuple);
    int offset = tuple.m_local_vid * 3 + tuple.m_local_eid;

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
        const long gvid = id(tuple, PrimitiveType::Vertex);
        const long geid = id(tuple, PrimitiveType::Edge);

        ConstAccessor<long> ff_accessor = create_const_accessor<long>(m_ff_handle);
        auto ff = ff_accessor.vector_attribute(tuple);

        long gcid_new = ff(tuple.m_local_eid);
        long lvid_new = -1, leid_new = -1;

        ConstAccessor<long> fv_accessor = create_const_accessor<long>(m_fv_handle);
        auto fv = fv_accessor.vector_attribute(gcid_new);

        ConstAccessor<long> fe_accessor = create_const_accessor<long>(m_fe_handle);
        auto fe = fe_accessor.vector_attribute(gcid_new);

        for (long i = 0; i < 3; ++i) {
            if (fe(i) == geid) {
                leid_new = i;
            }
            if (fv(i) == gvid) {
                lvid_new = i;
            }
        }
        assert(lvid_new != -1);
        assert(leid_new != -1);
        const Tuple res(
            lvid_new,
            leid_new,
            tuple.m_local_fid,
            gcid_new,
            get_cell_hash_slow(gcid_new));
        assert(is_valid(res));
        return res;
    }
    case PrimitiveType::Tetrahedron:
    default: throw std::runtime_error("Tuple switch: Invalid primitive type"); break;
    }
}

bool TriMesh::is_ccw(const Tuple& tuple) const
{
    assert(is_valid(tuple));
    int offset = tuple.m_local_vid * 3 + tuple.m_local_eid;
    return autogen::auto_2d_table_ccw[offset][0] == 1;
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
        static_cast<long>(VF.rows()),
        static_cast<long>(EF.rows()),
        static_cast<long>(FF.rows())};

    set_capacities(cap);


    // get Accessors for topology
    Accessor<long> fv_accessor = create_accessor<long>(m_fv_handle);
    Accessor<long> fe_accessor = create_accessor<long>(m_fe_handle);
    Accessor<long> ff_accessor = create_accessor<long>(m_ff_handle);
    Accessor<long> vf_accessor = create_accessor<long>(m_vf_handle);
    Accessor<long> ef_accessor = create_accessor<long>(m_ef_handle);

    Accessor<char> v_flag_accessor = get_flag_accessor(PrimitiveType::Vertex);
    Accessor<char> e_flag_accessor = get_flag_accessor(PrimitiveType::Edge);
    Accessor<char> f_flag_accessor = get_flag_accessor(PrimitiveType::Face);

    // iterate over the matrices and fill attributes
    for (long i = 0; i < capacity(PrimitiveType::Face); ++i) {
        fv_accessor.vector_attribute(i) = FV.row(i).transpose();
        fe_accessor.vector_attribute(i) = FE.row(i).transpose();
        ff_accessor.vector_attribute(i) = FF.row(i).transpose();

        f_flag_accessor.scalar_attribute(i) |= 0x1;
    }
    // m_vf
    for (long i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        vf_accessor.scalar_attribute(i) = VF(i);
        v_flag_accessor.scalar_attribute(i) |= 0x1;
    }
    // m_ef
    for (long i = 0; i < capacity(PrimitiveType::Edge); ++i) {
        ef_accessor.scalar_attribute(i) = EF(i);
        e_flag_accessor.scalar_attribute(i) |= 0x1;
    }
}

void TriMesh::initialize(Eigen::Ref<const RowVectors3l> F)
{
    auto [FE, FF, VF, EF] = trimesh_topology_initialization(F);
    initialize(F, FE, FF, VF, EF);
}

long TriMesh::_debug_id(const Tuple& tuple, PrimitiveType type) const
{
    // do not remove this warning!
    wmtk::logger().warn("This function must only be used for debugging!!");
    return id(tuple, type);
}

Tuple TriMesh::tuple_from_id(const PrimitiveType type, const long gid) const
{
    switch (type) {
    case PrimitiveType::Vertex: {
        return vertex_tuple_from_id(gid);
    }
    case PrimitiveType::Edge: {
        return edge_tuple_from_id(gid);
    }
    case PrimitiveType::Face: {
        return face_tuple_from_id(gid);
    }
    case PrimitiveType::Tetrahedron: {
        throw std::runtime_error("no tet tuple supported for trimesh");
        break;
    }
    default: throw std::runtime_error("Invalid primitive type"); break;
    }
}

Tuple TriMesh::vertex_tuple_from_id(long id) const
{
    ConstAccessor<long> vf_accessor = create_const_accessor<long>(m_vf_handle);
    auto f = vf_accessor.scalar_attribute(id);
    ConstAccessor<long> fv_accessor = create_const_accessor<long>(m_fv_handle);
    auto fv = fv_accessor.vector_attribute(f);
    for (long i = 0; i < 3; ++i) {
        if (fv(i) == id) {
            assert(autogen::auto_2d_table_complete_vertex[i][0] == i);
            const long leid = autogen::auto_2d_table_complete_vertex[i][1];
            Tuple v_tuple = Tuple(i, leid, -1, f, get_cell_hash_slow(f));
            assert(is_ccw(v_tuple));
            assert(is_valid(v_tuple));
            return v_tuple;
        }
    }
    throw std::runtime_error("vertex_tuple_from_id failed");
}

Tuple TriMesh::edge_tuple_from_id(long id) const
{
    ConstAccessor<long> ef_accessor = create_const_accessor<long>(m_ef_handle);
    auto f = ef_accessor.scalar_attribute(id);
    ConstAccessor<long> fe_accessor = create_const_accessor<long>(m_fe_handle);
    auto fe = fe_accessor.vector_attribute(f);
    for (long i = 0; i < 3; ++i) {
        if (fe(i) == id) {
            assert(autogen::auto_2d_table_complete_edge[i][1] == i);
            const long lvid = autogen::auto_2d_table_complete_edge[i][0];

            Tuple e_tuple = Tuple(lvid, i, -1, f, get_cell_hash_slow(f));
            assert(is_ccw(e_tuple));
            assert(is_valid(e_tuple));
            return e_tuple;
        }
    }
    throw std::runtime_error("edge_tuple_from_id failed");
}

Tuple TriMesh::face_tuple_from_id(long id) const
{
    Tuple f_tuple = Tuple(
        autogen::auto_2d_table_complete_vertex[0][0],
        autogen::auto_2d_table_complete_vertex[0][1],
        -1,
        id,
        get_cell_hash_slow(id)


    );
    assert(is_ccw(f_tuple));
    assert(is_valid(f_tuple));
    return f_tuple;
}

bool TriMesh::is_valid(const Tuple& tuple) const
{
    if (tuple.is_null()) return false;
    int offset = tuple.m_local_vid * 3 + tuple.m_local_eid;
    return tuple.m_local_vid >= 0 && tuple.m_local_eid >= 0 && tuple.m_global_cid >= 0 &&
           autogen::auto_2d_table_ccw[offset][0] >= 0;
}

bool TriMesh::is_outdated(const Tuple& tuple) const
{
    const long cid = id(tuple, PrimitiveType::Face);
    ConstAccessor<long> ha = get_cell_hash_accessor();
    return ha.scalar_attribute(cid) != tuple.m_hash;
}

bool TriMesh::is_connectivity_valid() const
{
    // get Accessors for topology
    ConstAccessor<long> fv_accessor = create_const_accessor<long>(m_fv_handle);
    ConstAccessor<long> fe_accessor = create_const_accessor<long>(m_fe_handle);
    ConstAccessor<long> ff_accessor = create_const_accessor<long>(m_ff_handle);
    ConstAccessor<long> vf_accessor = create_const_accessor<long>(m_vf_handle);
    ConstAccessor<long> ef_accessor = create_const_accessor<long>(m_ef_handle);
    ConstAccessor<char> v_flag_accessor = get_flag_accessor(PrimitiveType::Vertex);
    ConstAccessor<char> e_flag_accessor = get_flag_accessor(PrimitiveType::Edge);
    ConstAccessor<char> f_flag_accessor = get_flag_accessor(PrimitiveType::Face);

    // EF and FE
    for (long i = 0; i < capacity(PrimitiveType::Edge); ++i) {
        if (e_flag_accessor.scalar_attribute(i) == 0) {
            wmtk::logger().debug("Edge {} is deleted", i);
            continue;
        }
        int cnt = 0;
        for (long j = 0; j < 3; ++j) {
            if (fe_accessor.vector_attribute(ef_accessor.scalar_attribute(i))[j] == i) {
                cnt++;
            }
        }
        if (cnt != 1) {
            // std::cout << "EF and FE not compatible" << std::endl;
            return false;
        }
    }

    // VF and FV
    for (long i = 0; i < capacity(PrimitiveType::Vertex); ++i) {
        if (v_flag_accessor.scalar_attribute(i) == 0) {
            wmtk::logger().debug("Vertex {} is deleted", i);
            continue;
        }
        int cnt = 0;
        for (long j = 0; j < 3; ++j) {
            if (fv_accessor.vector_attribute(vf_accessor.scalar_attribute(i))[j] == i) {
                cnt++;
            }
        }
        if (cnt != 1) {
            // std::cout << "VF and FV not compatible" << std::endl;
            return false;
        }
    }

    // FE and EF
    for (long i = 0; i < capacity(PrimitiveType::Face); ++i) {
        if (f_flag_accessor.scalar_attribute(i) == 0) {
            wmtk::logger().debug("Face {} is deleted", i);
            continue;
        }
        for (long j = 0; j < 3; ++j) {
            long nb = ff_accessor.vector_attribute(i)[j];
            if (nb == -1) {
                if (ef_accessor.scalar_attribute(fe_accessor.vector_attribute(i)[j]) != i) {
                    // std::cout << "FF and FE not compatible" << std::endl;
                    return false;
                }
                continue;
            }

            int cnt = 0;
            int id_in_nb;
            for (long k = 0; k < 3; ++k) {
                if (ff_accessor.vector_attribute(nb)[k] == i) {
                    cnt++;
                    id_in_nb = k;
                }
            }

            if (cnt != 1) {
                // std::cout << "FF not valid" << std::endl;
                return false;
            }

            if (fe_accessor.vector_attribute(i)[j] != fe_accessor.vector_attribute(nb)[id_in_nb]) {
                // std::cout << "FF and FE not compatible" << std::endl;
                return false;
            }
        }
    }

    return true;
}

Tuple TriMesh::with_different_cid(const Tuple& t, long cid)
{
    Tuple r = t;
    r.m_global_cid = cid;
    return r;
}
} // namespace wmtk
