#include "Mesh.hpp"

namespace wmtk {

Tuple Mesh::tuple_from_cell(long cid) const
{
    return Tuple(0, 0, 0, cid, 0);
    // TODO: figure out how to compute hash
    // return Tuple(0,0,0,cid, hash(cid));
}

TriMesh::TriMesh()
    : m_vf_accessor(register_attribute_with_accessor<long>("m_vf", PrimitiveType::Vertex, 1))
    , m_ef_accessor(register_attribute_with_accessor<long>("m_ef", PrimitiveType::Edge, 1))
    , m_fv_accessor(register_attribute_with_accessor<long>("m_fv", PrimitiveType::Face, 3))
    , m_fe_accessor(register_attribute_with_accessor<long>("m_fe", PrimitiveType::Face, 3))
    , m_ff_accessor(register_attribute_with_accessor<long>("m_ff", PrimitiveType::Face, 3))
{}


TetMesh::TetMesh()
    : m_vt_accessor(register_attribute_with_accessor<long>("m_vt", PrimitiveType::Vertex, 1))
    , m_et_accessor(register_attribute_with_accessor<long>("m_et", PrimitiveType::Edge, 1))
    , m_ft_accessor(register_attribute_with_accessor<long>("m_ft", PrimitiveType::Face, 1))
    , m_tv_accessor(register_attribute_with_accessor<long>("m_tv", PrimitiveType::Tetrahedron, 4))
    , m_te_accessor(register_attribute_with_accessor<long>("m_te", PrimitiveType::Tetrahedron, 6))
    , m_tf_accessor(register_attribute_with_accessor<long>("m_tf", PrimitiveType::Tetrahedron, 4))
    , m_tt_accessor(register_attribute_with_accessor<long>("m_tt", PrimitiveType::Tetrahedron, 4))
{}


void trimesh_topology_initialization(
    Eigen::Ref<const RowVectors3l>& F,
    Eigen::Ref<const RowVectors3l>& FE,
    Eigen::Ref<const RowVectors3l>& FF,
    Eigen::Ref<const VectorXl>& VF,
    Eigen::Ref<const VectorXl>& EF)
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
            long v1 = std::static_cast<long>(F(f, i));
            long v2 = std::static_cast<long>(F(f, (i + 1) % F.cols()));
            if (v1 > v2) std::swap(v1, v2);
            std::vector<long> r(4);
            r[0] = v1;
            r[1] = v2;
            r[2] = f;
            r[3] = i;
            TTT[f] = r;
            FV(f, i) = v1;
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

void tetmesh_topology_initialization(
    Eigen::Ref<const RowVectors3d>& V,
    Eigen::Ref<const RowVectors4l>& F,
    TetMesh& mesh)
{}
} // namespace wmtk
