#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>

namespace wmtk::components::internal {
auto get_vf(const TriMesh& trimesh)
{
    wmtk::utils::EigenMatrixWriter writer;

    trimesh.serialize(writer);

    std::vector<double> vV;
    std::vector<uint32_t> vF;

    MatrixX<int64_t> F;
    MatrixX<double> V;
    writer.get_FV_matrix(F);
    writer.get_position_matrix(V);

    const uint32_t npts = V.rows();
    const uint32_t ntri = F.rows();
    vV.resize(V.size());
    vF.resize(F.size());
    Eigen::MatrixX<double>::MapType VT(vV.data(), 3, npts);
    Eigen::MatrixX<uint32_t>::MapType FT(vF.data(), 3, ntri);
    VT = V.transpose();
    FT = F.transpose().cast<uint32_t>();
    return std::make_tuple(std::make_pair(vV, npts), std::make_pair(vF, ntri));
}
} // namespace wmtk::components::internal
