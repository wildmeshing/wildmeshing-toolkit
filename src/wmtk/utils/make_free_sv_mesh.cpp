#include "make_free_sv_mesh.hpp"
#include <numeric>


namespace wmtk {

std::tuple<MatrixXl, VectorXl> make_free_sv_mesh(Eigen::Ref<const MatrixXl> S)
{
    MatrixXl F(S.rows(), S.cols());

    std::iota(F.data(), F.data() + F.size(), 0);

    VectorXl R(F.size());

    for (int j = 0; j < F.rows(); ++j) {
        for (int k = 0; k < F.cols(); ++k) {
            R(S(j, k)) = F(j, k);
        }
    }

    return {F, R};
}
// reindices the vertices
std::tuple<MatrixXl, Eigen::MatrixXd> make_free_sv_mesh_with_positions(
    Eigen::Ref<const MatrixXl> S,
    Eigen::Ref<const Eigen::MatrixXd> V)
{
    auto [F, R] = make_free_sv_mesh(S);

    return {F, V(R, Eigen::all)};
}

} // namespace wmtk
