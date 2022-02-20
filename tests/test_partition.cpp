#include <igl/write_triangle_mesh.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Partitioning.h>

#include <catch2/catch.hpp>
#include "spdlog/common.h"

#include <igl/read_triangle_mesh.h>

using namespace wmtk;

TEST_CASE("partition-mesh", "[]")
{
    Eigen::MatrixXd V;
    Eigen::MatrixXd F;
    std::string input_path = WMT_DATA_DIR "/bunny.off";
    igl::read_triangle_mesh(input_path, V, F);
    
    auto partitioned_v = partition_mesh_vertices(F, 10);
    REQUIRE(partitioned_v.size() == V.rows());
    REQUIRE(partitioned_v.maxCoeff() == 9);
}
