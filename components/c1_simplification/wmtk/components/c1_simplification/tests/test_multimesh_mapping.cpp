#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>

#include <wmtk/components/c1_simplification/c1_multimesh.hpp>
#include <wmtk/components/c1_simplification/c1_utils.hpp>

#include <catch2/catch_test_macros.hpp>
#include "spdlog/common.h"

using namespace wmtk;
using namespace components::c1_simplification;

TEST_CASE("multimesh_mapping", "[c1_simplification]")
{
    wmtk::logger().info("init");
    Eigen::MatrixXd SV, UVV, TV;
    Eigen::MatrixXi SF, UVF, TT;

    SV.resize(5, 3);
    SV << 0, 0, 0, //
        1, 0, 0, //
        0, 1, 0, //
        -1, 0, 0, //
        0, 0, 1;

    TV.resize(5, 3);
    TV << 0, 0, 0, //
        1, 0, 0, //
        0, 1, 0, //
        -1, 0, 0, //
        0, 0, 1;

    UVV.resize(7, 2);
    UVV << 0, 0, //
        1, 0, //
        1, 1, //
        -1, 0, //
        0, -1, //
        1, 1, //
        -1, 1;

    SF.resize(6, 3);
    SF << 1, 0, 2, //
        0, 3, 2, //
        0, 1, 4, //
        0, 4, 3, //
        4, 1, 2, //
        4, 2, 3; //

    UVF.resize(6, 3);
    UVF << 1, 0, 2, //
        0, 3, 2, //
        0, 1, 4, //
        0, 4, 3, //
        5, 1, 2, //
        6, 2, 3; //

    TT.resize(2, 4);
    TT << 0, 1, 2, 4, //
        0, 2, 3, 4;

    wmtk::logger().info("init tetmesh");

    // MMTetMesh tetmesh;
    // tetmesh.init_from_eigen(TV, TT);
    std::shared_ptr<MMTetMesh> tetmesh_ptr = std::make_shared<MMTetMesh>();
    tetmesh_ptr->init_from_eigen(TV, TT);
    wmtk::logger().info("initized tetmesh");

    std::shared_ptr<MMUVMesh> uvmesh_ptr = std::make_shared<MMUVMesh>();
    uvmesh_ptr->init_from_eigen(UVV, UVF);
    wmtk::logger().info("initized uvmesh");

    std::map<int64_t, int64_t> s2t_vid_map;
    std::vector<Vector3d> vgrads;
    for (int i = 0; i < 5; ++i) {
        s2t_vid_map[i] = i;
        vgrads.push_back(Vector3d(0, 0, 0));
    }
    std::vector<std::array<Vector3d, 3>> egrads;
    for (int i = 0; i < 9; ++i) {
        egrads.push_back({{Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0)}});
    }

    MMSurfaceMesh surfacemesh(tetmesh_ptr, uvmesh_ptr, 0.01);
    wmtk::logger().info("initized surfacemesh");
    surfacemesh.init_from_eigen_with_map_and_dofs(SV, SF, s2t_vid_map, vgrads, egrads);

    wmtk::logger().info("initized multimesh");

    wmtk::logger().info("test map to uv vertex");

    TriMesh::Tuple sv_t = surfacemesh.tuple_from_vertex(4);
    auto uv_v_ts = surfacemesh.map_to_uv_vertex_tuples(sv_t);
    for (size_t i = 0; i < uv_v_ts.size(); ++i) {
        // std::cout << uv_v_ts[i].vid(*surfacemesh.uvmesh_ptr);

        bool flag = uv_v_ts[i].vid(*surfacemesh.uvmesh_ptr) == 4 ||
                    uv_v_ts[i].vid(*surfacemesh.uvmesh_ptr) == 5 ||
                    uv_v_ts[i].vid(*surfacemesh.uvmesh_ptr) == 6;
        REQUIRE(flag == true);
    }
    // std::cout << std::endl;

    auto t_v_t = surfacemesh.map_to_tet_vertex_tuple(sv_t);
    REQUIRE(t_v_t.vid(*surfacemesh.tetmesh_ptr) == 4);

    TriMesh::Tuple se_t = surfacemesh.tuple_from_edge(4, 2, 4);
    auto vvv = surfacemesh.oriented_tri_vids(4);
    std::cout << vvv[0] << " " << vvv[1] << " " << vvv[2] << std::endl;
    std::cout << se_t.vid(surfacemesh) << std::endl;

    auto uv_e_ts = surfacemesh.map_to_uv_edge_tuples(se_t);

    REQUIRE(uv_e_ts.size() == 2);
    REQUIRE(uv_e_ts[0].vid(*uvmesh_ptr) == 5);
    REQUIRE(uv_e_ts[0].switch_vertex(*uvmesh_ptr).vid(*uvmesh_ptr) == 2);
    REQUIRE(uv_e_ts[1].vid(*uvmesh_ptr) == 6);
    REQUIRE(uv_e_ts[1].switch_vertex(*uvmesh_ptr).vid(*uvmesh_ptr) == 2);

    auto t_e_ts = surfacemesh.map_to_tet_edge_tuple(se_t);
    REQUIRE(t_e_ts.vid(*tetmesh_ptr) == 4);
    REQUIRE(t_e_ts.switch_vertex(*tetmesh_ptr).vid(*tetmesh_ptr) == 2);
}
