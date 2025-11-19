// #include <wmtk/TetMesh.h>
// #include <wmtk/TriMesh.h>

// #include <wmtk/components/c1_simplification/c1_multimesh.hpp>
// #include <wmtk/components/c1_simplification/c1_utils.hpp>

// #include <catch2/catch_test_macros.hpp>
// #include "spdlog/common.h"

// using namespace wmtk;
// using namespace components::c1_simplification;

// TEST_CASE("multimesh_collapse", "[c1_simplification]")
// {
//     wmtk::logger().info("init");
//     Eigen::MatrixXd SV, UVV, TV;
//     Eigen::MatrixXi SF, UVF, TT;

//     SV.resize(5, 3);
//     SV << 0, 0, 0, //
//         1, 0, 0, //
//         0, 1, 0, //
//         -1, 0, 0, //
//         0, 0, 1;

//     TV.resize(5, 3);
//     TV << 0, 0, 0, //
//         1, 0, 0, //
//         0, 1, 0, //
//         -1, 0, 0, //
//         0, 0, 1;

//     UVV.resize(7, 2);
//     UVV << 0, 0, //
//         1, 0, //
//         1, 1, //
//         -1, 0, //
//         0, -1, //
//         1, 1, //
//         -1, 1;

//     SF.resize(6, 3);
//     SF << 1, 0, 2, //
//         0, 3, 2, //
//         0, 1, 4, //
//         0, 4, 3, //
//         4, 1, 2, //
//         4, 2, 3; //

//     UVF.resize(6, 3);
//     UVF << 1, 0, 2, //
//         0, 3, 2, //
//         0, 1, 4, //
//         0, 4, 3, //
//         5, 1, 2, //
//         6, 2, 3; //

//     TT.resize(2, 4);
//     TT << 0, 1, 2, 4, //
//         0, 2, 3, 4;

//     wmtk::logger().info("init tetmesh");

//     // MMTetMesh tetmesh;
//     // tetmesh.init_from_eigen(TV, TT);
//     std::shared_ptr<MMTetMesh> tetmesh_ptr = std::make_shared<MMTetMesh>();
//     tetmesh_ptr->init_from_eigen(TV, TT);
//     wmtk::logger().info("initized tetmesh");

//     std::shared_ptr<MMUVMesh> uvmesh_ptr = std::make_shared<MMUVMesh>();
//     uvmesh_ptr->init_from_eigen(UVV, UVF);
//     wmtk::logger().info("initized uvmesh");

//     std::map<int64_t, int64_t> s2t_vid_map;
//     std::vector<Vector3d> vgrads;
//     for (int i = 0; i < 5; ++i) {
//         s2t_vid_map[i] = i;
//         vgrads.push_back(Vector3d(0, 0, 0));
//     }
//     std::vector<std::array<Vector3d, 3>> egrads;
//     for (int i = 0; i < 9; ++i) {
//         egrads.push_back({{Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0)}});
//     }

//     MMSurfaceMesh surfacemesh(tetmesh_ptr, uvmesh_ptr, 0.01);
//     wmtk::logger().info("initized surfacemesh");
//     // surfacemesh.init_from_eigen_with_map_and_dofs(SV, SF, s2t_vid_map, vgrads, egrads);

//     wmtk::logger().info("initized multimesh");

//     wmtk::logger().info("executing collapse");
//     auto se_to_collapse = surfacemesh.tuple_from_edge(3, 0, 1);
//     bool suc = surfacemesh.multimesh_collapse_edge(se_to_collapse);

//     REQUIRE(suc);
//     REQUIRE(tetmesh_ptr->get_tets().size() == 1);
//     auto tet_after = tetmesh_ptr->get_tets()[0];
//     auto tet_after_verts = tetmesh_ptr->oriented_tet_vids(tet_after);
//     std::cout << tet_after_verts[0] << " " << tet_after_verts[1] << " " << tet_after_verts[2] <<
//     " "
//               << tet_after_verts[3] << std::endl;
//     REQUIRE(tet_after_verts[0] == 0);
//     REQUIRE(tet_after_verts[1] == 1);
//     REQUIRE(tet_after_verts[2] == 2);
//     REQUIRE(tet_after_verts[3] == 4);

//     REQUIRE(uvmesh_ptr->get_faces().size() == 4);

//     REQUIRE(surfacemesh.get_faces().size() == 4);
// }
