#include <AdaptiveTessellation.h>
#include <igl/remove_duplicate_vertices.h>
#include <catch2/catch.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include "Collapse.h"
#include "Split.h"
using namespace wmtk;
using namespace lagrange;
using namespace adaptive_tessellation;

TEST_CASE("paired collapse", "[myfail][.]")
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

#include <wmtk/utils/DisableWarnings.hpp>
    //////////// ======== seam edge split
    // acsii art diamond
    //               1            4
    //             /   |        |   \     
    //     cv3    / (2)|        |(1) \   cv2
    //           /     |     cv0| |pe1\    
    //          /      |        | |/   \   
    //        0(0)   f0|        |f1   (0)3
    //          \   /| |        |      /
    //           \pe2| |        |     /
    //            \(1) |cv1     |(2) /
    //             \   |        |   /
    //               2            5

#include <wmtk/utils/EnableWarnings.hpp>
    // lambda function for checking each face's mirror data
    auto check_all_faces_mirror_info = [&m]() -> bool {
        for (auto& f : m.get_faces()) {
            assert(f.is_valid(m));
            for (int j = 0; j < 3; j++) {
                wmtk::TriMesh::Tuple e = m.tuple_from_edge(f.fid(m), j);
                e = e.is_ccw(m) ? e : e.switch_vertex(m);
                auto mirror_opt = m.face_attrs[e.fid(m)].mirror_edges[e.local_eid(m)];
                wmtk::logger().info("primary edge {}", e.info());
                wmtk::logger().info("mirror edge has value {}", mirror_opt.has_value());
                if (mirror_opt.has_value()) {
                    auto mirror_tup = mirror_opt.value();
                    wmtk::logger().info("mirror edge {}", mirror_tup.info());
                    auto primary_from_data =
                        m.face_attrs[mirror_tup.fid(m)].mirror_edges[mirror_tup.local_eid(m)];
                    wmtk::logger().info(
                        "primary edge from data has value {}",
                        primary_from_data.has_value());
                    wmtk::logger().info(
                        "primary edge from data {}",
                        primary_from_data.value().info());
                    wmtk::logger().info(
                        "Correctness checks: primary from data has value: {} mirror matches "
                        "primary{}",
                        primary_from_data.has_value(),
                        e == primary_from_data.value());
                    if (!primary_from_data.has_value()) return false;
                    if (!(e == primary_from_data.value())) return false;
                }
            }
        }
        return true;
    };

    m.create_mesh_debug(V, F);
    m.mesh_parameters.m_ignore_embedding = true;
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
    REQUIRE(check_all_faces_mirror_info());


    // split a few times
    AdaptiveTessellationPairedSplitEdgeOperation op;
    op(m, primary_edge2);
    REQUIRE(m.vert_capacity() == 8);
    REQUIRE(m.tri_capacity() == 4);
    wmtk::TriMesh::Tuple primary_edge3 = wmtk::TriMesh::Tuple(0, 1, 0, m);
    AdaptiveTessellationPairedSplitEdgeOperation op2;
    op2(m, primary_edge3);
    REQUIRE(m.vert_capacity() == 9);
    REQUIRE(m.tri_capacity() == 6);
    wmtk::TriMesh::Tuple primary_edge4 = wmtk::TriMesh::Tuple(3, 2, 1, m);
    AdaptiveTessellationPairedSplitEdgeOperation op3;
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

    REQUIRE(check_all_faces_mirror_info());

#include <wmtk/utils/DisableWarnings.hpp>
    ////////// ======= interior edge collapse
    // acsii art diamond
    //                1          4
    //              /(2)/|      |(1)\ 
    //             /   | |      |    \ 9
    //            /f2 /f5|      | f6 /\     
    //     cv3   /    |  |   cv0|   /  \    cv2
    //          /    /   |      |  /f1  \   
    //        0(0)--8|--6|      |7----(0)3
    //          \    \<--|cv1   |       /
    //           \   |pe6|      |      /
    //            \ f0\f4|      | f3  /
    //             \   | |      |    /
    //              \(1)\|      |(2)/
    //                2           5
#include <wmtk/utils/EnableWarnings.hpp>

    REQUIRE(m.tri_capacity() == 7);
    wmtk::TriMesh::Tuple primary_edge6 = wmtk::TriMesh::Tuple(6, 1, 4, m);
    REQUIRE(m.is_seam_vertex(primary_edge6));
    REQUIRE(!m.is_boundary_edge(primary_edge6));
    REQUIRE(!m.edge_attrs[primary_edge6.eid(m)].curve_id.has_value());
    REQUIRE(primary_edge6.is_valid(m));
    AdaptiveTessellationPairedCollapseEdgeOperation op4;
    REQUIRE(op4.before(m, primary_edge6));
    auto retdata = op4.execute(m, primary_edge6);
    REQUIRE(retdata.success);
    REQUIRE(op4.after(m, retdata));
    {
        std::vector<size_t> affected_fids{{0, 1, 2, 3, 6}};
        for (size_t j = 0; j < retdata.new_tris.size(); ++j) {
            spdlog::info("{} {}", affected_fids[j], retdata.new_tris[j].fid(m));
        }
        auto modified_tris = op4.modified_triangles(m);
        CHECK(retdata.new_tris == modified_tris);
        CHECK(retdata.new_tris.size() == affected_fids.size());
        for (const auto& ftup : retdata.new_tris) {
            CHECK(ftup.is_valid(m));
            size_t fid = ftup.fid(m);
            CHECK(std::binary_search(affected_fids.begin(), affected_fids.end(), fid));
        }
    }

#include <wmtk/utils/DisableWarnings.hpp>
    // acsii art diamond
    //                1    4
    //              /(2) ||(1)\ 
    //             /     ||    \ 9
    //            /f2    || f6 /\     
    //           /       ||   /  \    
    //          /        ||  /f1  \   
    //        0(0)-----10||7----(0)3
    //          \  <---  ||       /
    //           \       ||      /
    //            \ f0   || f3  /
    //             \     ||    /
    //              \(1) ||(2)/
    //                2     5
#include <wmtk/utils/EnableWarnings.hpp>
    int face_cnt = 0;
    for (auto& f : m.get_faces()) {
        face_cnt++;
    }
    REQUIRE(face_cnt == 5);
    const auto& primary_edge6_opt = op4.collapse_edge.get_return_tuple_opt();
    REQUIRE(primary_edge6_opt.has_value());
    const auto& primary_edge6_ret = primary_edge6_opt.value();
    REQUIRE(primary_edge6_ret.is_valid(m));
    REQUIRE(primary_edge6_ret.vid(m) == 10);
    REQUIRE(m.vert_capacity() == 11);
    REQUIRE(primary_edge6_ret.switch_vertex(m).vid(m) == 0);
    REQUIRE(!m.is_seam_edge(primary_edge6_ret));
    auto swapped_pe6_ret = primary_edge6_ret.switch_edge(m);
    REQUIRE(m.edge_attrs[swapped_pe6_ret.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[swapped_pe6_ret.eid(m)].curve_id.value() == 1);
    REQUIRE(m.get_oriented_mirror_edge(swapped_pe6_ret).is_valid(m));
    REQUIRE(m.get_oriented_mirror_edge(swapped_pe6_ret).fid(m) == 3);
    REQUIRE(m.get_oriented_mirror_edge(swapped_pe6_ret).vid(m) == 5);
    REQUIRE(check_all_faces_mirror_info());
    spdlog::warn("2===========================");
    /////// debug
    for (auto& e : m.get_edges()) {
        REQUIRE(e.is_valid(m));
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
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 0);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 10);
                break;
            case 1:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 6);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 7);
                break;
            case 4:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 2);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 10);
                break;
            default: break;
            }
        }
    }

#include <wmtk/utils/DisableWarnings.hpp>
    ////////// ======= boundary edge collapse
    //                1    4
    //              /(2) ||(1)\ 
    //             /     ||    \ 9
    //            /f2    || f6 /\     
    //           /       ||   /|\\ pe7
    //          /        ||  /f1 \\   
    //        0(0)-----10||7----(0)3
    //          \        ||       /
    //           \       ||      /
    //            \ f0   || f3  /
    //             \     ||    /
    //              \(1) ||(2)/
    //                2     5
#include <wmtk/utils/EnableWarnings.hpp>
    wmtk::TriMesh::Tuple primary_edge7 = wmtk::TriMesh::Tuple(3, 2, 1, m);
    REQUIRE(primary_edge7.switch_vertex(m).vid(m) == 9);
    REQUIRE(!m.is_seam_vertex(primary_edge7));
    REQUIRE(m.is_boundary_edge(primary_edge7));
    REQUIRE(!m.is_seam_edge(primary_edge7));
    REQUIRE(m.edge_attrs[primary_edge7.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge7.eid(m)].curve_id.value() == 2);

    AdaptiveTessellationPairedCollapseEdgeOperation op5;
    op5(m, primary_edge7);
#include <wmtk/utils/DisableWarnings.hpp>
    // acsii art diamond
    //                1    4
    //              /(2) ||(1)\ 
    //             /     ||    \ 
    //            /f2    || f6|\\     
    //           /       ||     \\    
    //          /        ||      \\   
    //        0(0)-----10||7----(0)11
    //          \        ||       /
    //           \       ||      /
    //            \ f0   || f3  /
    //             \     ||    /
    //              \(1) ||(2)/
    //                2     5
#include <wmtk/utils/EnableWarnings.hpp>
    const auto& primary_edge7_opt = op5.collapse_edge.get_return_tuple_opt();
    REQUIRE(primary_edge7_opt.has_value());
    const auto& primary_edge7_ret = primary_edge7_opt.value();
    REQUIRE(primary_edge7_ret.is_valid(m));
    REQUIRE(primary_edge7_ret.vid(m) == 11);
    REQUIRE(primary_edge7_ret.switch_vertex(m).vid(m) == 4);

    {
        std::vector<size_t> affected_fids{{3, 6}};
        auto new_tris = op5.modified_triangles(m);
        CHECK(new_tris.size() == affected_fids.size());
        for (const auto& ftup : new_tris) {
            CHECK(ftup.is_valid(m));
            size_t fid = ftup.fid(m);
            CHECK(std::binary_search(affected_fids.begin(), affected_fids.end(), fid));
        }
    }
    int valid_verts_cnt = 0;
    for (auto& v : m.get_vertices()) {
        if (v.is_valid(m)) {
            valid_verts_cnt++;
        }
    }
    REQUIRE(valid_verts_cnt == 8);
    int valid_faces_cnt = 0;
    for (auto& f : m.get_faces()) {
        if (f.is_valid(m)) {
            valid_faces_cnt++;
        }
    }
    REQUIRE(valid_faces_cnt == 4);
    REQUIRE(!m.is_seam_edge(primary_edge7_ret));
    REQUIRE(!m.edge_attrs[primary_edge7_ret.switch_edge(m).eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge7_ret.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge7_ret.eid(m)].curve_id.value() == 2);
    // REQUIRE(!m.edge_attrs[primary_edge7_ret.eid(m)].curve_id.has_value());
    // REQUIRE(m.edge_attrs[primary_edge7_ret.switch_edge(m).eid(m)].curve_id.has_value());
    // REQUIRE(m.edge_attrs[primary_edge7_ret.switch_edge(m).eid(m)].curve_id.value() == 2);

    REQUIRE(check_all_faces_mirror_info());
    /////// debug
    for (auto& e : m.get_edges()) {
        REQUIRE(e.is_valid(m));
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
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 0);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 10);
                break;
            case 1:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 6);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 7);
                break;
            case 4:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 2);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 10);
                break;
            default: break;
            }
        }
    }
#include <wmtk/utils/DisableWarnings.hpp>
    ////////// ======= seam edge collapse
    // acsii art diamond
    //                1    4
    //              /(2) ||(1)\ 
    //             /     ||    \ 
    //            /f2    || f6  \     
    //           /       ||      \    
    //          /        ||       \   
    //        0(0)-----10||7----(0)11
    //          \        || |     /
    //           \       || |    /
    //            \ f0   || |/f3/
    //             \     ||    /
    //              \(1) ||(2)/
    //                2     5
#include <wmtk/utils/EnableWarnings.hpp>
    wmtk::TriMesh::Tuple primary_edge8 = wmtk::TriMesh::Tuple(7, 0, 3, m);
    REQUIRE(primary_edge8.switch_vertex(m).vid(m) == 5);
    {
        auto mirror_edge = m.get_oriented_mirror_edge(primary_edge8);

        REQUIRE(mirror_edge.vid(m) == 2);
        REQUIRE(mirror_edge.switch_vertex(m).vid(m) == 10);
    }
    REQUIRE(m.is_seam_vertex(primary_edge8));
    REQUIRE(m.is_seam_edge(primary_edge8));
    REQUIRE(m.edge_attrs[primary_edge8.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge8.eid(m)].curve_id.value() == 0);

    AdaptiveTessellationPairedCollapseEdgeOperation op6;
    REQUIRE(op6.before(m, primary_edge8));

    const auto mirror_input_opt = op6.get_mirror_edge_tuple_opt();
    REQUIRE(mirror_input_opt.has_value());
    const TriMeshTuple mirror_input = mirror_input_opt.value();
    spdlog::warn(mirror_input.info());
    REQUIRE(mirror_input.vid(m) == 10);
    REQUIRE(mirror_input.switch_vertex(m).vid(m) == 2);
    {
        auto retdata = op6.execute(m, primary_edge8);
        REQUIRE(retdata.success);
    }


#include <wmtk/utils/DisableWarnings.hpp>
    // acsii art diamond
    //                1    4
    //              /(2) ||(1)\ 
    //             /     ||    \ 9
    //            /      ||     \     
    //           /       ||      \    
    //          /    f2  ||  f6   \   
    //        0(0)    /| ||       (0)11
    //          \      | ||       /
    //           \  pe9| ||  pe8 /
    //            \    | ||  _  /
    //             \   | ||  / /
    //              \(1) ||(2)/
    //                13    12
#include <wmtk/utils/EnableWarnings.hpp>


    const auto& primary_edge8_opt = op6.collapse_edge.get_return_tuple_opt();
    const auto& primary_edge9_opt = op6.collapse_mirror_edge.get_return_tuple_opt();
    REQUIRE(primary_edge8_opt.has_value());
    REQUIRE(primary_edge9_opt.has_value());
    const auto& primary_edge8_ret = primary_edge8_opt.value();
    const auto& primary_edge9_ret = primary_edge9_opt.value();

    REQUIRE(primary_edge8_ret.vid(m) == 12);
    REQUIRE(primary_edge8_ret.switch_vertex(m).vid(m) == 11);

    REQUIRE(primary_edge9_ret.vid(m) == 13);
    REQUIRE(primary_edge9_ret.switch_vertex(m).vid(m) == 1);
    valid_verts_cnt = 0;
    for (auto& v : m.get_vertices()) {
        if (v.is_valid(m)) {
            valid_verts_cnt++;
        }
    }
    REQUIRE(valid_verts_cnt == 6);
    valid_faces_cnt = 0;
    for (auto& f : m.get_faces()) {
        if (f.is_valid(m)) {
            valid_faces_cnt++;
        }
    }

    {
        std::vector<size_t> affected_fids{{2, 6}};
        auto new_tris = op6.modified_triangles(m);
        CHECK(new_tris.size() == affected_fids.size());
        for (const auto& ftup : new_tris) {
            CHECK(ftup.is_valid(m));
            size_t fid = ftup.fid(m);
            CHECK(std::binary_search(affected_fids.begin(), affected_fids.end(), fid));
        }
    }
    REQUIRE(valid_faces_cnt == 2);
    REQUIRE(!m.is_seam_edge(primary_edge8_ret));
    REQUIRE(m.is_boundary_edge(primary_edge8_ret));
    REQUIRE(m.edge_attrs[primary_edge8_ret.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge8_ret.eid(m)].curve_id.value() == 2);
    REQUIRE(m.is_seam_edge(primary_edge8_ret.switch_edge(m)));
    REQUIRE(m.is_seam_vertex(primary_edge8_ret));
    const auto primary_edge8_se = primary_edge8_ret.switch_edge(m);
    REQUIRE(primary_edge8_se.vid(m) == 12);
    REQUIRE(primary_edge8_se.switch_vertex(m).vid(m) == 4);
    REQUIRE(m.is_seam_edge(primary_edge8_se));
    REQUIRE(m.is_boundary_edge(primary_edge8_se));
    REQUIRE(m.edge_attrs[primary_edge8_se.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge8_se.eid(m)].curve_id.value() == 0);
    REQUIRE(m.is_seam_vertex(primary_edge8_se));

    REQUIRE(m.is_seam_edge(primary_edge9_ret));
    REQUIRE(m.is_boundary_edge(primary_edge9_ret));
    REQUIRE(m.edge_attrs[primary_edge9_ret.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge9_ret.eid(m)].curve_id.value() == 1);
    REQUIRE(m.is_seam_vertex(primary_edge9_ret));
    const auto primary_edge9_se = primary_edge9_ret.switch_edge(m);
    REQUIRE(!m.is_seam_edge(primary_edge9_se));
    REQUIRE(m.is_boundary_edge(primary_edge9_se));
    REQUIRE(m.edge_attrs[primary_edge9_se.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge9_se.eid(m)].curve_id.value() == 3);
    REQUIRE(m.is_seam_edge(primary_edge9_se.switch_edge(m)));
    REQUIRE(m.is_seam_vertex(primary_edge9_se));

    REQUIRE(m.get_oriented_mirror_edge(primary_edge8_se).vid(m) == 1);
    REQUIRE(m.get_oriented_mirror_edge(primary_edge9_ret).vid(m) == 4);

    spdlog::info(primary_edge8_ret.info());
    spdlog::info(primary_edge9_ret.info());

    /////// debug
    for (auto& e : m.get_edges()) {
        REQUIRE(e.is_valid(m));
        if (m.is_boundary_edge(e)) {
            REQUIRE(m.edge_attrs[e.eid(m)].curve_id.has_value());
        } else {
            REQUIRE(!m.edge_attrs[e.eid(m)].curve_id.has_value());
        }
        if (m.is_seam_edge(e)) {
            switch (e.vid(m)) {
            case 12:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 2);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 1);
                break;
            case 13:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 6);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 4);
                break;
            case 1:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 6);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 12);
                break;
            case 4:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 2);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 13);
                break;
            default: break;
            }
        }
    }
}
