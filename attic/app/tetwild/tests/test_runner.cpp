#include <TetWild.h>

#include <wmtk/TetMesh.h>

#include <catch2/catch_test_macros.hpp>

#include <igl/read_triangle_mesh.h>

using namespace wmtk;
using namespace tetwild;

TEST_CASE("optimize_mesh", "[test_runner]")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/Octocat.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);
    REQUIRE(V.rows() == 18944);
    REQUIRE(F.rows() == 37884);
}
