#include <catch2/catch.hpp>

#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>
#include <wmtk/TetMesh.h>

#include <igl/doublearea.h>

using namespace wmtk;

auto harmonic_energy = [](const Eigen::MatrixXd& verts) -> double {
    Eigen::MatrixXi lF(4, 3);
    lF << 0, 1, 2, 0, 2, 3, 0, 1, 3, 1, 2, 3; // sorted local vids
    Eigen::Matrix3d v3 = verts.bottomRows(3).rowwise() - verts.row(0);
    auto vol = v3.determinant();
    if (vol < 0.) return std::numeric_limits<double>::max();
    Eigen::VectorXd dblarea;
    igl::doublearea(verts, lF, dblarea);
    dblarea /= 2.;
    auto sum_area2 = dblarea.squaredNorm();
    return sum_area2 / vol; // to be divided by (3*3) in the formula.
};

TEST_CASE("harmonic-tri-energy")
{
    Eigen::MatrixXd unit = Eigen::MatrixXd::Zero(4, 3);
    unit.bottomRows(3) = Eigen::MatrixXd::Identity(3, 3);
    auto ee = harmonic_energy(unit);
    REQUIRE(ee == 1.5);
}