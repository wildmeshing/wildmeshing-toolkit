#include <catch2/catch.hpp>

#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>
#include <wmtk/TetMesh.h>

#include <igl/doublearea.h>
#include <wmtk/utils/TetraQualityUtils.hpp>

using namespace wmtk;


TEST_CASE("harmonic-tet-energy")
{
    Eigen::MatrixXd unit = Eigen::MatrixXd::Zero(4, 3);
    unit.bottomRows(3) = Eigen::MatrixXd::Identity(3, 3);
    auto ee = wmtk::harmonic_energy(unit);
    REQUIRE(ee == 1.5);
}

TEST_CASE("harmonic-tet-swaps") {

}