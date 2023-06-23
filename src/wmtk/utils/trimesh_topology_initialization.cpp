#include <wmtk/trimesh_topology_initialization.h>
namespace wmtk {
namespace {

void trimesh_topology_initialization(
    Eigen::Ref<const Mesh::RowVectors3l> F,
    Eigen::Ref<Mesh::RowVectors3l> FE,
    Eigen::Ref<Mesh::RowVectors3l> FF,
    Eigen::Ref<Mesh::VectorXl> VF,
    Eigen::Ref<Mesh::VectorXl> EF)
{
    assert(F.cols()==3);
    std::vector<std::vector<long>> TTT;
    FE.resize(F.rows(), 3);
    FF.resize(F.rows(), 3);

    long vertex_count = F.coeffWise().max();
    VF.resize(vertex_count, 1, -1);
    
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
