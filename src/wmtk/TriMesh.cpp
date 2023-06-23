#include <Mesh.hpp>

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

long TriMesh::id(const Tuple& tuple, const PrimitiveType& type) const override
{
    switch (type) {
    case PrimitiveType::Vertex: return m_fv[tuple.m_global_cid * 3 + tuple.m_local_vid]; break;
    case PrimitiveType::Edge: return m_fe[tuple.m_global_cid * 3 + tuple.m_local_eid]; break;
    case PrimitiveType::Triangle: return tuple.m_global_cid; break;
    default: throw std::runtime_error("Tuple id: Invalid primitive type");
    }
}

Tuple TriMesh::switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override
{
    bool ccw = is_ccw(tuple);
    switch (type) {
    case PrimitiveType::Vertex:
        return Tuple(
            (tuple.m_local_vid + ccw ? 1 : 2) % 3,
            tuple.m_local_eid,
            tuple.m_local_fid,
            tuple.m_global_cid,
            tuple.m_hash);

    case PrimitiveType::Edge:
        return Tuple(
            tuple.m_local_vid,
            (tuple.m_local_eid + ccw ? 2 : 1) % 3,
            tuple.m_local_fid,
            tuple.m_global_cid,
            tuple.m_hash);
    case PrimitiveType::Triangle: {
        long gvid = id(tuple, 0);
        long geid = id(tuple, 1);
        long gcid_new = m_ff[tuple.m_global_cid * 3 + tuple.m_local_eid];
        long lvid_new, leid_new;
        for (long i = 0; i < 3; ++i) {
            if (m_fe[gcid_new * 3 + i] == geid) {
                leid_new = m_fe[gcid_new * 3 + i];
            }
            if (m_fv[gcid_new * 3 + i] == gvid) {
                lvid_new = m_fv[gcid_new * 3 + i];
            }
        }
        return Tuple(lvid_new, leid_new, tuple.m_local_fid, gcid_new, tuple.m_hash);
    }
    default: throw std::runtime_error("Tuple switch: Invalid primitive type"); break;
    }
}

bool TriMesh::is_ccw(const Tuple& tuple) const override
{
    if (m_fv[tuple.m_global_cid * 3 + (tuple.m_local_eid + 1) % 3] == id(tuple, 0))
        return true;
    else
        return false;
}

void trimesh_topology_initialization(
    Eigen::Ref<const Mesh::RowVectors3l> F,
    Eigen::Ref<Mesh::RowVectors3l> FV,
    Eigen::Ref<Mesh::RowVectors3l> FE,
    Eigen::Ref<Mesh::RowVectors3l> FF,
    Eigen::Ref<Mesh::VectorXl> VF,
    Eigen::Ref<Mesh::VectorXl> EF)
{
    std::vector<std::vector<long>> TTT;
    FV.resize(F.rows(), F.cols());
    FE.resize(F.rows(), F.cols());
    FF.resize(F.rows(), F.cols());
    VF.resize(F.rows(), 1);
    EF.resize(F.rows(), 1);
    TTT.resize(F.rows(), std::vector<long>(4));
    for (int f = 0; f < F.rows(); ++f) {
        for (int i = 0; i < F.cols(); ++i) {
            // v1 v2 f ei
            long v1 = F(f, i);
            long v2 = F(f, (i + 1) % F.cols());
            if (v1 > v2) std::swap(v1, v2);
            std::vector<long> r(4);
            r[0] = v1;
            r[1] = v2;
            r[2] = f;
            r[3] = i;
            TTT[f] = r;
            // FV(f, i) = v1;
        }
    }
    std::sort(TTT.begin(), TTT.end());

    // iterate over TTT to initialize topology
    // assumption is the same edge is always next to each other in the sorted TTT
    int unique_edges = 0;
    long v01 = TTT[0][0];
    long v02 = TTT[0][1];
    long f0 = TTT[0][2];
    long e0 = TTT[0][3];
    FE(f0, e0) = unique_edges;
    VF(v01, 0) = f0;
    VF(v02, 0) = f0;
    EF(unique_edges, 0) = f0;

    for (int i = 1; i < TTT.size(); ++i) {
        int va1 = TTT[i][0];
        int va2 = TTT[i][1];
        int fa = TTT[i][2];
        int eia = TTT[i][3];

        int vb1 = TTT[i - 1][0];
        int vb2 = TTT[i - 1][1];
        int fb = TTT[i - 1][2];
        int eib = TTT[i - 1][3];
        if (va1 == vb1 & va2 == vb2) {
            // same edge
            FF(fa, eia) = fb;
            FF(fb, eib) = fa;
            continue;
        } else {
            unique_edges++;
            FE(fa, eia) = unique_edges;
            VF(va1, 0) = fa;
            VF(va2, 0) = fa;
            EF(unique_edges, 0) = fa;
            FF(fa, eia) = -1;
        }
    }
}
} // namespace wmtk