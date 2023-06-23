#include <wmtk/utils/trimesh_topology_initialization.h>
#include <wmtk/Mesh.hpp>

#include <catch2/catch.hpp>

#include <igl/read_triangle_mesh.h>

#include <stdlib.h>
#include <iostream>

using namespace wmtk;

TEST_CASE("load mesh from libigl and test mesh topology", "[test_topology_2d]")
{
    Eigen::MatrixXd V;
    Eigen::Matrix<long int, -1, -1> F;

    igl::read_triangle_mesh(
        "/mnt/ff7e01f4-89ad-40d7-a113-112e01c9fd93/siqi/wildmeshing/data/fan.obj",
        V,
        F);
    Eigen::Ref<RowVectors3l> RefF = Eigen::Ref<RowVectors3l>{F};

    Eigen::Matrix<long int, -1, -1> FV, FE, FF, VF, EF;
    // trimesh_topology_initialization(F, FV, FE, FF, VF, EF);

    std::cout << "FV: " << FV << std::endl;
}