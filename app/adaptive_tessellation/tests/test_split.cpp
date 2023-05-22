#include <AdaptiveTessellation.h>
#include <igl/remove_duplicate_vertices.h>
#include <catch2/catch.hpp>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include "Split.h"
using namespace wmtk;
using namespace lagrange;
using namespace adaptive_tessellation;


TEST_CASE("paired split")
{
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
    /////// debug
    for (auto& e : m.get_edges()) {
        if (m.is_boundary_edge(e)) {
            REQUIRE(m.edge_attrs[e.eid(m)].curve_id.has_value());
        } else {
            REQUIRE(!m.edge_attrs[e.eid(m)].curve_id.has_value());
        }
    }
    AdaptiveTessellationPairedSplitEdgeOperation op;
    op(m, primary_edge2);

    // acsii art diamond
    //                1    4
    //              /(2)||(1)\ 
    //             /    ||    \     
    //            /  f2 ||  f1 \     
    //           /      || |pe1 \    
    //          /       || |/    \   
    //        0(0)---- 6||7 ---(0)3
    //          \    /| ||       /
    //           \ pe2| ||      /
    //            \  f0 || f3  /
    //             \    ||    /
    //              \(1)||(2)/
    //                2    5

    REQUIRE(m.vert_capacity() == 8);
    REQUIRE(m.tri_capacity() == 4);
    primary_edge2 = op.split_edge.return_edge_tuple;
    primary_edge1 = op.mirror_split_edge.return_edge_tuple;
    // checking for first face
    REQUIRE(m.face_attrs[0].mirror_edges[0].has_value());
    REQUIRE(m.face_attrs[0].mirror_edges[0].value().fid(m) == 3);
    REQUIRE(m.face_attrs[0].mirror_edges[0].value().vid(m) == 7);
    REQUIRE(m.get_oriented_mirror_edge(primary_edge2).fid(m) == 3);
    REQUIRE(m.get_oriented_mirror_edge(primary_edge2).vid(m) == 7);
    // checking for second face
    REQUIRE(m.face_attrs[1].mirror_edges[0].has_value());
    REQUIRE(m.face_attrs[1].mirror_edges[0].value().fid(m) == 2);
    REQUIRE(m.face_attrs[1].mirror_edges[0].value().vid(m) == 6);
    REQUIRE(m.get_oriented_mirror_edge(primary_edge1).fid(m) == 2);
    REQUIRE(m.get_oriented_mirror_edge(primary_edge1).vid(m) == 6);
    // checking for second face mirroring face
    REQUIRE(m.face_attrs[2].mirror_edges[0].has_value());
    REQUIRE(m.face_attrs[2].mirror_edges[0].value().fid(m) == 1);
    REQUIRE(m.face_attrs[2].mirror_edges[0].value().vid(m) == 4);
    // checking for first face mirroring face
    REQUIRE(m.face_attrs[3].mirror_edges[0].has_value());
    REQUIRE(m.face_attrs[3].mirror_edges[0].value().fid(m) == 0);
    REQUIRE(m.face_attrs[3].mirror_edges[0].value().vid(m) == 2);

    // checking curve id
    REQUIRE(m.edge_attrs[primary_edge1.eid(m)].curve_id.value() == 0);
    REQUIRE(m.edge_attrs[primary_edge2.eid(m)].curve_id.value() == 1);
    REQUIRE(m.edge_attrs[m.get_oriented_mirror_edge(primary_edge1).eid(m)].curve_id.value() == 1);
    REQUIRE(m.edge_attrs[m.get_oriented_mirror_edge(primary_edge2).eid(m)].curve_id.value() == 0);
    /////// debug
    for (auto& e : m.get_edges()) {
        if (m.is_boundary_edge(e)) {
            REQUIRE(m.edge_attrs[e.eid(m)].curve_id.has_value());
        } else {
            REQUIRE(!m.edge_attrs[e.eid(m)].curve_id.has_value());
        }
    }
    {
        std::vector<size_t> affected_fids{{0,1,2,3}};
        auto new_tris = op.modified_triangles(m);
        CHECK(new_tris.size() == affected_fids.size());
        for (const auto& ftup : new_tris) {
            CHECK(ftup.is_valid(m));
            size_t fid = ftup.fid(m);
            CHECK(std::binary_search(affected_fids.begin(), affected_fids.end(), fid));
        }
    }

#include <wmtk/utils/DisableWarnings.hpp>
    //////////======= interior edge split
    // acsii art diamond
    //                1    4
    //              /(2)||(1)\ 
    //             /    ||    \     
    //            /  f2 ||  f1 \     
    //           /      ||      \    
    //          /       ||       \   
    //        0(0)---- 6||7 ---(0)3
    //          \ ----> ||       /
    //           \  pe3 ||      /
    //            \  f0 || f3  /
    //             \    ||    /
    //              \(1)||(2)/
    //                2    5
#include <wmtk/utils/EnableWarnings.hpp>
    wmtk::TriMesh::Tuple primary_edge3 = wmtk::TriMesh::Tuple(0, 1, 0, m);
    REQUIRE(!m.edge_attrs[primary_edge3.eid(m)].curve_id.has_value());
    REQUIRE(!m.is_seam_edge(primary_edge3));
    REQUIRE(!m.is_boundary_edge(primary_edge3));
    AdaptiveTessellationPairedSplitEdgeOperation op2;
    op2(m, primary_edge3);
#include <wmtk/utils/DisableWarnings.hpp>
    // acsii art diamond
    //                1    4
    //              /(2)/||(1)\ 
    //             /   | ||    \     
    //            /f2 /f5||  f1 \     
    //           /    |  ||      \    
    //          /    /   ||       \   
    //        0(0)--8|--6||7----(0)3
    //          \ -->\   ||       /
    //           \   |   ||      /
    //            \ f0\f4|| f3  /
    //             \   | ||    /
    //              \(1)\||(2)/
    //                2     5
#include <wmtk/utils/EnableWarnings.hpp>
    REQUIRE(m.vert_capacity() == 9);
    REQUIRE(m.tri_capacity() == 6);

    primary_edge3 = op2.split_edge.return_edge_tuple;
    REQUIRE(!m.is_seam_edge(primary_edge3));
    REQUIRE(!m.is_boundary_edge(primary_edge3));
    REQUIRE(!m.edge_attrs[primary_edge3.eid(m)].curve_id.has_value());
    REQUIRE(!m.edge_attrs[primary_edge3.eid(m)].curve_id.has_value());
    REQUIRE(primary_edge3.vid(m) == 0);
    REQUIRE(primary_edge3.fid(m) == 0);
    REQUIRE(primary_edge3.switch_vertex(m).vid(m) == 8);
    /////// debug
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
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 1);
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
    {
        std::vector<size_t> affected_fids{{0,2,4,5}};
        auto new_tris = op2.modified_triangles(m);
        CHECK(new_tris.size() == affected_fids.size());
        for (const auto& ftup : new_tris) {
            CHECK(ftup.is_valid(m));
            size_t fid = ftup.fid(m);
            CHECK(std::binary_search(affected_fids.begin(), affected_fids.end(), fid));
        }
    }

#include <wmtk/utils/DisableWarnings.hpp>
    ////////// ======= boundary edge split
    // acsii art diamond
    //                1    4
    //              /(2)/||(1)\ 
    //             /   | ||    \     
    //            /f2 /f5||  f1 \     
    //           /    |  ||   |\ \    
    //          /    /   ||  pe4\ \   
    //        0(0)--8|--6||7----(0)3
    //          \    \   ||       /
    //           \   |   ||      /
    //            \ f0\f4|| f3  /
    //             \   | ||    /
    //              \(1)\||(2)/
    //                2     5
#include <wmtk/utils/EnableWarnings.hpp>
    wmtk::TriMesh::Tuple primary_edge4 = wmtk::TriMesh::Tuple(3, 2, 1, m);
    REQUIRE(m.edge_attrs[primary_edge4.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge4.eid(m)].curve_id.value() == 2);
    REQUIRE(m.is_boundary_edge(primary_edge4));
    REQUIRE(!m.is_seam_edge(primary_edge4));
    AdaptiveTessellationPairedSplitEdgeOperation op3;
    op3(m, primary_edge4);
#include <wmtk/utils/DisableWarnings.hpp>
    // acsii art diamond
    //                1    4
    //              /(2)/||(1)\ 
    //             /   | ||    \ 9
    //            /f2 /f5|| f6 /\     
    //           /    |  ||   /|\\    
    //          /    /   ||  /f1 \\   
    //        0(0)--8|--6||7----(0)3
    //          \    \   ||       /
    //           \   |   ||      /
    //            \ f0\f4|| f3  /
    //             \   | ||    /
    //              \(1)\||(2)/
    //                2     5
#include <wmtk/utils/EnableWarnings.hpp>

    REQUIRE(m.vert_capacity() == 10);
    REQUIRE(m.tri_capacity() == 7);
    primary_edge4 = op3.split_edge.return_edge_tuple;
    REQUIRE(m.edge_attrs[primary_edge4.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge4.eid(m)].curve_id.value() == 2);
    REQUIRE(m.is_boundary_edge(primary_edge4));
    REQUIRE(!m.is_seam_edge(primary_edge4));
    wmtk::TriMesh::Tuple primary_edge5 = op3.split_edge.return_edge_tuple;
    primary_edge5 =
        primary_edge5.switch_vertex(m).switch_edge(m).switch_face(m).value().switch_edge(m);
    REQUIRE(m.edge_attrs[primary_edge5.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge5.eid(m)].curve_id.value() == 2);
    REQUIRE(m.is_boundary_edge(primary_edge5));
    REQUIRE(!m.is_seam_edge(primary_edge5));
    /////// debug
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

    {
        std::vector<size_t> affected_fids{{1,6}};
        auto new_tris = op3.modified_triangles(m);
        CHECK(new_tris.size() == affected_fids.size());
        for (const auto& ftup : new_tris) {
            CHECK(ftup.is_valid(m));
            size_t fid = ftup.fid(m);
            CHECK(std::binary_search(affected_fids.begin(), affected_fids.end(), fid));
        }
    }
}
