#include <AdaptiveTessellation.h>
#include <igl/remove_duplicate_vertices.h>
#include <catch2/catch.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include "Split.h"
#include "Swap.h"
using namespace wmtk;
using namespace lagrange;
using namespace adaptive_tessellation;

TEST_CASE("paired swap")
{
    Eigen::MatrixXd V(6, 2);
    Eigen::MatrixXi F(2, 3);
    V.row(0) << -1., 0.;
    V.row(1) << 0., 1.;
    V.row(2) << 0., -1;
    V.row(3) << 1., 0;
    V.row(4) << 0., 1.;
    V.row(5) << 0., -1.;
    F.row(0) << 0, 2, 1;
    F.row(1) << 3, 4, 5;
    AdaptiveTessellation m;

    m.create_mesh_debug(V, F);
    m.mesh_parameters.m_ignore_embedding = true;
    m.mesh_parameters.m_do_not_output = true;
    // set up mesh
    wmtk::TriMesh::Tuple primary_edge1 = wmtk::TriMesh::Tuple(4, 0, 1, m);
    wmtk::TriMesh::Tuple primary_edge2 = wmtk::TriMesh::Tuple(2, 0, 0, m);
    primary_edge1 = primary_edge1.is_ccw(m) ? primary_edge1 : primary_edge1.switch_vertex(m);
    primary_edge2 = primary_edge2.is_ccw(m) ? primary_edge2 : primary_edge2.switch_vertex(m);
    m.face_attrs[0].mirror_edges[0] = std::make_optional<wmtk::TriMesh::Tuple>(primary_edge1);
    m.face_attrs[1].mirror_edges[0] = std::make_optional<wmtk::TriMesh::Tuple>(primary_edge2);
    m.edge_attrs[primary_edge1.eid(m)].curve_id = std::make_optional<int>(0);
    m.edge_attrs[primary_edge2.eid(m)].curve_id = std::make_optional<int>(1);
    m.edge_attrs[primary_edge1.switch_edge(m).eid(m)].curve_id = std::make_optional<int>(2);
    m.edge_attrs[primary_edge1.switch_vertex(m).switch_edge(m).eid(m)].curve_id =
        std::make_optional<int>(2);
    m.edge_attrs[primary_edge2.switch_edge(m).eid(m)].curve_id = std::make_optional<int>(3);
    m.edge_attrs[primary_edge2.switch_vertex(m).switch_edge(m).eid(m)].curve_id =
        std::make_optional<int>(3);
    // split a few times
    AdaptiveTessellationPairedEdgeSplitOperation op;
    op(m, primary_edge2);
    REQUIRE(m.vert_capacity() == 8);
    REQUIRE(m.tri_capacity() == 4);
    wmtk::TriMesh::Tuple primary_edge3 = wmtk::TriMesh::Tuple(0, 1, 0, m);
    AdaptiveTessellationPairedEdgeSplitOperation op2;
    op2(m, primary_edge3);
    REQUIRE(m.vert_capacity() == 9);
    REQUIRE(m.tri_capacity() == 6);
    wmtk::TriMesh::Tuple primary_edge4 = wmtk::TriMesh::Tuple(3, 2, 1, m);
    AdaptiveTessellationPairedEdgeSplitOperation op3;
    op3(m, primary_edge4);
    /////// make sure it is valid mesh
    REQUIRE(m.vert_capacity() == 10);
    REQUIRE(m.tri_capacity() == 7);
    for (auto& e : m.get_edges()) {
        if (m.is_boundary_edge(e)) {
            REQUIRE(m.edge_attrs[e.eid(m)].curve_id.has_value());
        } else {
            REQUIRE(!m.edge_attrs[e.eid(m)].curve_id.has_value());
        }
        if (m.is_seam_edge(e)) {
            switch (e.vid(m)) {
            case 2:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 3);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 7);
                break;
            case 5:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 4);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 6);
                break;
            case 1:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 6);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 7);
                break;
            case 4:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 5);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 6);
                break;
            default: break;
            }
        }
    }
#include <wmtk/utils/DisableWarnings.hpp>
    ////////// ======= interior edge swap
    // acsii art diamond
    //                1          4
    //              /(2)/|      |(1)\ 
    //             /   | |      |    \ 9
    //            /f2 /f5|      | f6 /\     
    //     cv3   /    |  |   cv0| /|/  \    cv2
    //          /    /   |      |/ / f1 \   
    //        0(0)--8|--6|      |7----(0)3
    //          \    \   |cv1   |       /
    //           \   |   |      |      /
    //            \ f0\f4|      | f3  /
    //             \   | |      |    /
    //              \(1)\|      |(2)/
    //                2           5
#include <wmtk/utils/EnableWarnings.hpp>
    wmtk::TriMesh::Tuple primary_edge5 = wmtk::TriMesh::Tuple(7, 1, 6, m);
    REQUIRE(primary_edge5.is_valid(m));
    REQUIRE(primary_edge5.vid(m) == 7);
    REQUIRE(primary_edge5.switch_vertex(m).vid(m) == 9);
    AdaptiveTessellationEdgeSwapOperation op4;
    // this operation should fail because of it would generate colinear traingles
    op4(m, primary_edge5);
    std::vector<TriMeshTuple> op4_modified_triangles = op4.modified_triangles(m);
    REQUIRE(!op4);
    REQUIRE(op4_modified_triangles.size() == 0);
    {
        std::vector<size_t> affected_fids{};
        auto new_tris = op4.modified_triangles(m);
        CHECK(new_tris.size() == affected_fids.size());
        for (const auto& ftup : new_tris) {
            CHECK(ftup.is_valid(m));
            size_t fid = ftup.fid(m);
            CHECK(std::binary_search(affected_fids.begin(), affected_fids.end(), fid));
        }
    }

#include <wmtk/utils/DisableWarnings.hpp>
    ////////// ======= interior edge swap
    // acsii art diamond
    //                1          4
    //              /(2)/|      |(1)\ 
    //             /   | |      |    \ 9
    //            /f2 /f5|      | f6 /\     
    //     cv3   /    |  |   cv0|   /  \    cv2
    //          /    /   |      |  / f1 \   
    //        0(0)--8|--6|      |7----(0)3
    //          \    \   |cv1   |  ---> /
    //           \   |   |      |  pe6 /
    //            \ f0\f4|      | f3  /
    //             \   | |      |    /
    //              \(1)\|      |(2)/
    //                2           5
#include <wmtk/utils/EnableWarnings.hpp>

    wmtk::TriMesh::Tuple primary_edge6 = wmtk::TriMesh::Tuple(7, 2, 3, m);
    REQUIRE(primary_edge6.is_valid(m));
    REQUIRE(primary_edge6.vid(m) == 7);
    REQUIRE(primary_edge6.switch_vertex(m).vid(m) == 3);
    AdaptiveTessellationEdgeSwapOperation op5;
    // this operation should fail because of it would generate colinear traingles
    op5(m, primary_edge6);
    std::vector<TriMeshTuple> op5_modified_triangles = op5.modified_triangles(m);
    {
        std::vector<size_t> affected_fids{1, 3};
        auto new_tris = op5.modified_triangles(m);
        CHECK(new_tris.size() == affected_fids.size());
        for (const auto& ftup : new_tris) {
            CHECK(ftup.is_valid(m));
            size_t fid = ftup.fid(m);
            CHECK(std::binary_search(affected_fids.begin(), affected_fids.end(), fid));
        }
    }

#include <wmtk/utils/DisableWarnings.hpp>
    // acsii art diamond
    //                1          4
    //              /(2)/|      |(1)\ 
    //             /   | |      |    \ 9
    //            /f2 /f5|      | f6 /\     
    //     cv3   /    |  |   cv0|   // \    cv2
    //          /    /   |      |  / |  \   
    //        0(0)--8|--6|      |7   ||\(0)3
    //          \    \   |cv1   |    ||  /
    //           \   |   |      |   / | /
    //            \ f0\f4|      |f3| f1/
    //             \   | |      |  |  /
    //              \(1)\|      |(2)/
    //                2           5
#include <wmtk/utils/EnableWarnings.hpp>

    REQUIRE(op5_modified_triangles.size() == 2);
    const wmtk::TriMesh::Tuple op5_ret = op5.get_return_tuple_opt().value();
    REQUIRE(op5_ret.is_valid(m));
    REQUIRE(op5_ret.vid(m) == 5);
    // edge 3-5
    REQUIRE(m.is_boundary_edge(op5_ret.switch_edge(m)));
    REQUIRE(m.edge_attrs[op5_ret.switch_edge(m).eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[op5_ret.switch_edge(m).eid(m)].curve_id.value() == 2);
    // edge 5-7
    REQUIRE(m.is_seam_edge(op5_modified_triangles[1].switch_edge(m)));
    REQUIRE(m.edge_attrs[op5_modified_triangles[1].switch_edge(m).eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[op5_modified_triangles[1].switch_edge(m).eid(m)].curve_id.value() == 0);
    REQUIRE(m.get_oriented_mirror_edge(op5_modified_triangles[1].switch_edge(m)).vid(m) == 6);
    REQUIRE(m.get_oriented_mirror_edge(op5_modified_triangles[1].switch_edge(m)).fid(m) == 4);
    //////
    REQUIRE(m.vert_capacity() == 10);
    REQUIRE(m.tri_capacity() == 7);
    for (auto& e : m.get_edges()) {
        if (m.is_boundary_edge(e)) {
            REQUIRE(m.edge_attrs[e.eid(m)].curve_id.has_value());
        } else {
            REQUIRE(!m.edge_attrs[e.eid(m)].curve_id.has_value());
        }
        if (m.is_seam_edge(e)) {
            switch (e.vid(m)) {
            case 2:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 3);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 7);
                break;
            case 5:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 4);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 6);
                break;
            case 1:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 6);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 7);
                break;
            case 4:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 5);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 6);
                break;
            default: break;
            }
        }
    }

#include <wmtk/utils/DisableWarnings.hpp>
    // acsii art diamond
    //                1          4
    //              /(2)/|      |(1)\ 
    //             /   | |      |    \ 9
    //            /f2 /f5|      | f6 /\     
    //     cv3   /    |  |   cv0|   // \    cv2
    //          /    /   |      |  / |  \   
    //        0(0)--8|--6|      |7   ||\(0)3
    //          \    \   |cv1   |    ||  /
    //           \   |   |      |   / | /
    //            \ f0\f4|      |f3| f1/
    //             \   | |      |  |  /
    //              \(1)\|      |(2)/
    //                2           5
    ////////// ======= seam edge swap
    /// should be rejected. not swapping seam edge
#include <wmtk/utils/EnableWarnings.hpp>
    wmtk::TriMesh::Tuple primary_edge7 = wmtk::TriMesh::Tuple(5, 0, 3, m);
    REQUIRE(primary_edge7.is_valid(m));
    REQUIRE(primary_edge7.vid(m) == 5);
    REQUIRE(primary_edge7.switch_vertex(m).vid(m) == 7);
    REQUIRE(m.is_seam_edge(primary_edge7));
    AdaptiveTessellationEdgeSwapOperation op6;
    op6(m, primary_edge7);
    std::vector<TriMeshTuple> op6_modified_triangles = op6.modified_triangles(m);
    REQUIRE(!op6);
    REQUIRE(op6_modified_triangles.size() == 0);

    ////////// ======= boundary edge swap
    /// should be rejected. don't swap boundary
    wmtk::TriMesh::Tuple primary_edge8 = wmtk::TriMesh::Tuple(2, 2, 0, m);
    REQUIRE(primary_edge8.is_valid(m));
    REQUIRE(primary_edge8.vid(m) == 2);
    REQUIRE(primary_edge8.switch_vertex(m).vid(m) == 0);
    REQUIRE(m.is_boundary_edge(primary_edge8));
    AdaptiveTessellationEdgeSwapOperation op7;
    op7(m, primary_edge8);
    REQUIRE(!op7);
    std::vector<TriMeshTuple> op7_modified_triangles = op7.modified_triangles(m);
    REQUIRE(op7_modified_triangles.size() == 0);
}
