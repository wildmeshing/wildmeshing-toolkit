#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/EigenMatrixWriter.hpp>

#include <igl/remove_duplicate_vertices.h>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal {
std::tuple<std::pair<std::vector<double>, uint32_t>, std::pair<std::vector<uint32_t>, uint32_t>>
get_vf(const TriMesh& trimesh)
{
    wmtk::utils::EigenMatrixWriter writer;

    trimesh.serialize(writer);

    std::vector<double> vV;
    std::vector<uint32_t> vF;

    MatrixX<int64_t> F_tmp;
    MatrixX<double> V_tmp;
    writer.get_FV_matrix(F_tmp);
    writer.get_position_matrix(V_tmp);

    // debug code
    MatrixX<int64_t> F;
    MatrixX<double> V;

    Eigen::VectorXi IV, _;
    igl::remove_duplicate_vertices(V_tmp, F_tmp, 0, V, IV, _, F);

    wmtk::logger().info("removed duplicated vertices {} -> {}", V_tmp.rows(), V.rows());

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
