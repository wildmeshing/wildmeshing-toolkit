#include <wmtk/utils/trimesh_topology_initialization.h>
#include <wmtk/Mesh.hpp>

#include <catch2/catch.hpp>

#include <igl/read_triangle_mesh.h>

#include <stdlib.h>
#include <iostream>

using namespace wmtk;

TEST_CASE("load mesh from libigl and test mesh topology on a single triangle", "[test_topology_single_triangle]")
{
    Eigen::Matrix<long, 1, 3> F;
    F << 0, 1, 2;
    auto [FE, FF, VF, EF] = trimesh_topology_initialization(F);

    std::cout << "F: " << F << std::endl;
    std::cout << "FE: " << FE << std::endl;
    std::cout << "FF: " << FF << std::endl;
    std::cout << "VF: " << VF << std::endl;
    std::cout << "EF: " << EF << std::endl;

    // 1. Test relationship between EF and FE
    for (size_t i = 0; i < EF.size(); ++i)
    {
        CHECK((FE.row(EF(i)).array() == i).any());
    }

    // 2. Test relationship between VF and F
    for (size_t i = 0; i < VF.size(); ++i)
    {
        CHECK((F.row(VF(i)).array() == i).any());
    }
    
    // 3. Test relationship between FF and FE
    for (size_t i = 0; i < FF.rows(); ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            long nb = FF(i, j);
            if (nb < 0) 
                continue;
            
            CHECK((FF.row(nb).array() == i).any());

            if ((FF.row(nb).array() == i).any())
            {
                int cnt = (FF.row(nb).array() == i).count();
                CHECK(cnt == 1);

                auto is_nb = (FE.row(nb).array() == i);
                for (size_t k = 0; k < 3; ++k)
                {
                    if (is_nb(k))
                    {
                        CHECK(FE(i, j) == FE(nb, k));
                    }
                }
            }    
        }
    }
}

TEST_CASE("load mesh from libigl and test mesh topology", "[test_topology_2D]")
{
    Eigen::MatrixXd V;
    Eigen::Matrix<long, -1, -1> F;

    igl::read_triangle_mesh(WMTK_DATA_DIR "/fan.obj", V, F);

    auto [FE, FF, VF, EF] = trimesh_topology_initialization(F);

    // std::cout << "F: " << F << std::endl;
    // std::cout << "FE: " << FE << std::endl;
    // std::cout << "FF: " << FF << std::endl;
    // std::cout << "VF: " << VF << std::endl;
    // std::cout << "EF: " << EF << std::endl;

    // 1. Test relationship between EF and FE
    for (size_t i = 0; i < EF.size(); ++i)
    {
        CHECK((FE.row(EF(i)).array() == i).any());
    }

    // 2. Test relationship between VF and F
    for (size_t i = 0; i < VF.size(); ++i)
    {
        CHECK((F.row(VF(i)).array() == i).any());
    }
    
    // 3. Test relationship between FF and FE
    for (size_t i = 0; i < FF.rows(); ++i)
    {
        for (size_t j = 0; j < 3; ++j)
        {
            long nb = FF(i, j);
            if (nb < 0) 
                continue;
            
            CHECK((FF.row(nb).array() == i).any());

            if ((FF.row(nb).array() == i).any())
            {
                int cnt = (FF.row(nb).array() == i).count();
                CHECK(cnt == 1);

                auto is_nb = (FE.row(nb).array() == i);
                for (size_t k = 0; k < 3; ++k)
                {
                    if (is_nb(k))
                    {
                        CHECK(FE(i, j) == FE(nb, k));
                    }
                }
            }    
        }
    }
}