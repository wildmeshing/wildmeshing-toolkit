#include <catch2/catch_test_macros.hpp>

#include <numeric>
#include <wmtk/Accessor.hpp>
#include <wmtk/TriMesh_operations.hpp>
#include <wmtk/utils/Logger.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"
#include <wmtk/SyncTriMesh.hpp>


using namespace wmtk;
using namespace wmtk::tests;

using TM = TriMesh;

TEST_CASE("try init")
{
    REQUIRE(true);
    RowVectors3l tris_main;
    tris_main.resize(3, 3);
    tris_main.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
    tris_main.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
    tris_main.row(0) = Eigen::Matrix<long, 3, 1>{0, 1, 2};
    
}
