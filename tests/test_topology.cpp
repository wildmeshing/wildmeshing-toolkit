#include "tools/DEBUG_TetMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"

#include <wmtk/utils/edgemesh_topology_initialization.h>
#include <wmtk/utils/tetmesh_topology_initialization.h>
#include <wmtk/utils/trimesh_topology_initialization.h>

#include <wmtk/Mesh.hpp>
#include <wmtk/io/MeshReader.hpp>

#include <catch2/catch_test_macros.hpp>

#include <stdlib.h>
#include <iostream>

#include <wmtk/utils/Logger.hpp>

using namespace wmtk;

TEST_CASE("topology_of_single_triangle", "[topology][2D]")
{
    Eigen::Matrix<int64_t, 1, 3> F;
    F << 0, 1, 2;
    auto [FE, FF, VF, EF] = trimesh_topology_initialization(F);

    // std::cout << "F:\n" << F << std::endl;
    // std::cout << "FE:\n" << FE << std::endl;
    // std::cout << "FF:\n" << FF << std::endl;
    // std::cout << "VF:\n" << VF << std::endl;
    // std::cout << "EF:\n" << EF << std::endl;

    // 1. Test relationship between EF and FE
    for (int i = 0; i < EF.size(); ++i) {
        CHECK((FE.row(EF(i)).array() == i).any());
    }

    // 2. Test relationship between VF and F
    for (int i = 0; i < VF.size(); ++i) {
        CHECK((F.row(VF(i)).array() == i).any());
    }

    // 3. Test relationship between FF and FE
    for (int i = 0; i < FF.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            int64_t nb = FF(i, j);
            if (nb < 0) continue;

            CHECK((FF.row(nb).array() == i).any());

            if ((FF.row(nb).array() == i).any()) {
                int cnt = (FF.row(nb).array() == i).count();
                CHECK(cnt == 1);

                auto is_nb = (FF.row(nb).array() == i);
                for (int k = 0; k < 3; ++k) {
                    if (is_nb(k)) {
                        CHECK(FE(i, j) == FE(nb, k));
                    }
                }
            }
        }
    }
}

TEST_CASE("topology_of_two_triangles", "[topology][2D]")
{
    Eigen::Matrix<int64_t, 2, 3> F;
    F << 0, 1, 2, 1, 3, 2;

    auto [FE, FF, VF, EF] = trimesh_topology_initialization(F);

    // std::cout << "F:\n" << F << std::endl;
    // std::cout << "FE:\n" << FE << std::endl;
    // std::cout << "FF:\n" << FF << std::endl;
    // std::cout << "VF:\n" << VF << std::endl;
    // std::cout << "EF:\n" << EF << std::endl;

    // 1. Test relationship between EF and FE
    for (int i = 0; i < EF.size(); ++i) {
        CHECK((FE.row(EF(i)).array() == i).any());
    }

    // 2. Test relationship between VF and F
    for (int i = 0; i < VF.size(); ++i) {
        CHECK((F.row(VF(i)).array() == i).any());
    }

    // 3. Test relationship between FF and FE
    for (int i = 0; i < FF.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            int64_t nb = FF(i, j);
            if (nb < 0) continue;

            CHECK((FF.row(nb).array() == i).any());

            if ((FF.row(nb).array() == i).any()) {
                int cnt = (FF.row(nb).array() == i).count();
                CHECK(cnt == 1);

                auto is_nb = (FF.row(nb).array() == i);
                for (int k = 0; k < 3; ++k) {
                    if (is_nb(k)) {
                        CHECK(FE(i, j) == FE(nb, k));
                    }
                }
            }
        }
    }
}

TEST_CASE("topology_of_complex_meshes", "[topology][2D]")
{
    Eigen::Matrix<int64_t, -1, -1> F;

    std::vector<std::string> names = {
        "/Octocat.msh",
        "/armadillo.msh",
        "/blub.msh",
        // "/bunny.msh",
        "/circle.msh",
        "/fan.msh",
        "/sphere.msh",
        "/test_triwild.msh",
        "/hemisphere.msh"};

    for (auto name : names) {
        std::string path;
        path.append(WMTK_DATA_DIR);
        path.append(name);
        auto tmp = read_mesh(path);
        const auto& mesh = static_cast<wmtk::tests::DEBUG_TriMesh&>(*tmp);
        const auto& tris = mesh.get_all(PrimitiveType::Triangle);
        F.resize(tris.size(), 3);
        for (size_t i = 0; i < tris.size(); ++i) {
            const auto& t = tris[i];
            auto t1 = mesh.switch_tuple(t, PrimitiveType::Vertex);
            auto t2 =
                mesh.switch_tuple(mesh.switch_tuple(t, PrimitiveType::Edge), PrimitiveType::Vertex);

            int64_t vid0 = mesh.id(t, PrimitiveType::Vertex);
            int64_t vid1 = mesh.id(t1, PrimitiveType::Vertex);
            int64_t vid2 = mesh.id(t2, PrimitiveType::Vertex);
            F.row(i) << vid0, vid1, vid2;
        }

        auto [FE, FF, VF, EF] = trimesh_topology_initialization(F);

        // std::cout << "F:\n" << F << std::endl;
        // std::cout << "FE:\n" << FE << std::endl;
        // std::cout << "FF:\n" << FF << std::endl;
        // std::cout << "VF:\n" << VF << std::endl;
        // std::cout << "EF:\n" << EF << std::endl;

        // 1. Test relationship between EF and FE
        for (int i = 0; i < EF.size(); ++i) {
            CHECK((FE.row(EF(i)).array() == i).any());
        }

        // 2. Test relationship between VF and F
        for (int i = 0; i < VF.size(); ++i) {
            if (VF(i) < 0) continue;
            CHECK((F.row(VF(i)).array() == i).any());
        }

        // 3. Test relationship between FF and FE
        for (int i = 0; i < FF.rows(); ++i) {
            for (int j = 0; j < 3; ++j) {
                int64_t nb = FF(i, j);
                if (nb < 0) continue;

                CHECK((FF.row(nb).array() == i).any());

                if ((FF.row(nb).array() == i).any()) {
                    int cnt = (FF.row(nb).array() == i).count();
                    CHECK(cnt == 1);

                    auto is_nb = (FF.row(nb).array() == i);
                    for (int k = 0; k < 3; ++k) {
                        if (is_nb(k)) {
                            // wmtk::logger().info("{} {} {} {}", i, j, nb, k);
                            CHECK(FE(i, j) == FE(nb, k));
                        }
                    }
                }
            }
        }
    }
}

TEST_CASE("topology_of_two_adjacent_tets", "[topology][3D]")
{
    // Two tetrahedra are sharing one face
    // there are 7 unique faces and 9 unique edges

    Eigen::Matrix<int64_t, 2, 4> T;
    T << 0, 1, 2, 3, 1, 2, 3, 4;

    auto [TE, TF, TT, VT, ET, FT] = tetmesh_topology_initialization(T);

    // 1. Test the maximum in TE and TF
    CHECK(TE.maxCoeff() == 9 - 1);
    CHECK(TF.maxCoeff() == 7 - 1);
    CHECK(TT.maxCoeff() == 1);

    // 2. Test the relationship between ET and TE
    for (int i = 0; i < ET.size(); ++i) {
        CHECK((TE.row(ET(i)).array() == i).any());
    }

    // 3. Test the relationship between FT and TF
    for (int i = 0; i < FT.size(); ++i) {
        CHECK((TF.row(FT(i)).array() == i).any());
    }

    // 4. Test the relationship between VT and T
    for (int i = 0; i < VT.size(); ++i) {
        if (VT(i) < 0) continue;
        CHECK((T.row(VT(i)).array() == i).any());
    }

    // 5. Test the relationship between TT and TF and TE
    for (int i = 0; i < TT.rows(); ++i) {
        for (int j = 0; j < 4; ++j) {
            int64_t nb = TT(i, j);
            if (nb < 0) continue;

            CHECK((TT.row(nb).array() == i).any());

            if ((TT.row(nb).array() == i).any()) {
                int cnt = (TT.row(nb).array() == i).count();
                CHECK(cnt == 1);

                auto is_nb = (TT.row(nb).array() == i);
                for (int k = 0; k < 4; ++k) {
                    if (is_nb(k)) {
                        // wmtk::logger().info("{} {} {} {}", i, j, nb, k);
                        CHECK(TF(i, j) == TF(nb, k));
                    }
                }
            }
        }
    }
}

TEST_CASE("topology_of_two_independent_tets", "[topology][3D]")
{
    // Two tetrahedra not sharing anything
    // there are 8 unique faces and 12 unique edges

    Eigen::Matrix<int64_t, 2, 4> T;
    T << 0, 1, 2, 3, 4, 5, 6, 7;

    auto [TE, TF, TT, VT, ET, FT] = tetmesh_topology_initialization(T);

    // 1. Test the maximum in TE and TF
    CHECK(TE.maxCoeff() == 12 - 1);
    CHECK(TF.maxCoeff() == 8 - 1);
    CHECK(TT.maxCoeff() == -1);

    // 2. Test the relationship between ET and TE
    for (int i = 0; i < ET.size(); ++i) {
        CHECK((TE.row(ET(i)).array() == i).any());
    }

    // 3. Test the relationship between FT and TF
    for (int i = 0; i < FT.size(); ++i) {
        CHECK((TF.row(FT(i)).array() == i).any());
    }

    // 4. Test the relationship between VT and T
    for (int i = 0; i < VT.size(); ++i) {
        if (VT(i) < 0) continue;
        CHECK((T.row(VT(i)).array() == i).any());
    }

    // 5. Test the relationship between TT and TF and TE
    for (int i = 0; i < TT.rows(); ++i) {
        for (int j = 0; j < 4; ++j) {
            int64_t nb = TT(i, j);
            if (nb < 0) continue;

            CHECK((TT.row(nb).array() == i).any());

            if ((TT.row(nb).array() == i).any()) {
                int cnt = (TT.row(nb).array() == i).count();
                CHECK(cnt == 1);

                auto is_nb = (TT.row(nb).array() == i);
                for (int k = 0; k < 4; ++k) {
                    if (is_nb(k)) {
                        // wmtk::logger().info("{} {} {} {}", i, j, nb, k);
                        CHECK(TF(i, j) == TF(nb, k));
                    }
                }
            }
        }
    }
}

TEST_CASE("topology_of_tet_bunny", "[topology][3D]")
{
    logger().set_level(spdlog::level::warn);

    auto tmp = read_mesh(WMTK_DATA_DIR "/bunny_3d.msh");
    Eigen::Matrix<int64_t, -1, -1> T;
    const auto& mesh = static_cast<wmtk::tests_3d::DEBUG_TetMesh&>(*tmp);
    const auto& tets = mesh.get_all(PrimitiveType::Tetrahedron);
    T.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i) {
        const auto& t = tets[i];
        auto t1 = mesh.switch_tuple(t, PrimitiveType::Vertex);
        auto t2 =
            mesh.switch_tuple(mesh.switch_tuple(t, PrimitiveType::Edge), PrimitiveType::Vertex);
        auto t3 = mesh.switch_tuple(
            mesh.switch_tuple(mesh.switch_tuple(t, PrimitiveType::Triangle), PrimitiveType::Edge),
            PrimitiveType::Vertex);
        int64_t vid0 = mesh.id(t, PrimitiveType::Vertex);
        int64_t vid1 = mesh.id(t1, PrimitiveType::Vertex);
        int64_t vid2 = mesh.id(t2, PrimitiveType::Vertex);
        int64_t vid3 = mesh.id(t3, PrimitiveType::Vertex);
        T.row(i) << vid0, vid1, vid2, vid3;
    }

    auto [TE, TF, TT, VT, ET, FT] = tetmesh_topology_initialization(T);

    // 1. Test the maximum in TE and TF
    CHECK(TE.maxCoeff() == (ET.size() - 1));
    CHECK(TF.maxCoeff() == (FT.size() - 1));

    // 2. Test the relationship between ET and TE
    for (int i = 0; i < ET.size(); ++i) {
        CHECK((TE.row(ET(i)).array() == i).any());
    }

    // 3. Test the relationship between FT and TF
    for (int i = 0; i < FT.size(); ++i) {
        CHECK((TF.row(FT(i)).array() == i).any());
    }

    // 4. Test the relationship between VT and T
    for (int i = 0; i < VT.size(); ++i) {
        if (VT(i) < 0) continue;
        CHECK((T.row(VT(i)).array() == i).any());
    }

    // 5. Test the relationship between TT and TF and TE
    for (int i = 0; i < TT.rows(); ++i) {
        for (int j = 0; j < 4; ++j) {
            int64_t nb = TT(i, j);
            if (nb < 0) continue;

            CHECK((TT.row(nb).array() == i).any());

            if ((TT.row(nb).array() == i).any()) {
                int cnt = (TT.row(nb).array() == i).count();
                CHECK(cnt == 1);

                auto is_nb = (TT.row(nb).array() == i);
                for (int k = 0; k < 4; ++k) {
                    if (is_nb(k)) {
                        // wmtk::logger().info("{} {} {} {}", i, j, nb, k);
                        CHECK(TF(i, j) == TF(nb, k));
                    }
                }
            }
        }
    }
}

TEST_CASE("topology_test_1d", "[topology][1D]")
{
    Eigen::Matrix<int64_t, -1, 2> E;
    SECTION("single_line")
    {
        /*
            0 ---- 1
        */
        E.resize(1, 2);
        E << 0, 1;
    }
    SECTION("multiple_lines")
    {
        /*
            5 -- 2 -- 0 -- 1 -- 4 -- 3
        */
        E.resize(5, 2);
        E << 0, 1, 1, 4, 3, 4, 2, 0, 5, 2;
    }
    SECTION("loop_lines")
    {
        /*
            5 -- 2 -- 0 -- 1 -- 4 -- 3 -- 5*
        */
        E.resize(6, 2);
        E << 0, 1, 1, 4, 3, 4, 2, 0, 5, 2, 5, 3;
    }
    SECTION("two_line_loop")
    {
        /*
            0 -- 1 -- 0*
        */
        E.resize(2, 2);
        E << 0, 1, 1, 0;
    }
    SECTION("self_loop")
    {
        /*
            0 -- 0*
        */
        E.resize(1, 2);
        E << 0, 0;
    }

    const auto [EE, VE] = edgemesh_topology_initialization(E);

    // 1. Test relationship between VE and EV
    for (int i = 0; i < VE.size(); ++i) {
        CHECK((E.row(VE(i)).array() == i).any());
    }

    // 2. Test relationship between EV and EE
    for (int i = 0; i < EE.rows(); ++i) {
        for (int j = 0; j < 2; ++j) {
            int64_t nb = EE(i, j);
            if (nb < 0) continue;

            CHECK((EE.row(nb).array() == i).any());

            // TODO add checks
        }
    }
}
