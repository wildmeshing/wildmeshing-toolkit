#pragma once

#include <AdaptiveTessellation.h>
#include <igl/facet_components.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/writeOBJ.h>
#include <lagrange/IndexedAttribute.h>
#include <lagrange/attribute_names.h>
#include <lagrange/foreach_attribute.h>
#include <lagrange/io/load_mesh.h>
#include <lagrange/triangulate_polygonal_facets.h>
#include <lagrange/utils/fpe.h>
#include <lagrange/views.h>
#include <remeshing/UniformRemeshing.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <wmtk/utils/BoundaryParametrization.h>
#include <wmtk/utils/Image.h>
#include <wmtk/utils/MipMap.h>
#include <wmtk/utils/autodiff.h>
#include <wmtk/utils/bicubic_interpolation.h>
#include <catch2/catch.hpp>
#include <finitediff.hpp>
#include <functional>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include "Collapse.h"
#include "Smooth.h"
#include "Split.h"
#include "Swap.h"
using namespace wmtk;
using namespace lagrange;
using namespace adaptive_tessellation;

template <class T>
using RowMatrix2 = Eigen::Matrix<T, Eigen::Dynamic, 2, Eigen::RowMajor>;
using Index = uint64_t;
using Scalar = double;

TEST_CASE("AABB")
{
    const std::string root(WMTK_DATA_DIR);
    const std::string path = root + "/test_triwild.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);
    AdaptiveTessellation m;
    m.create_mesh(V, F);
    m.set_projection();

    auto result = m.mesh_parameters.m_get_closest_point(Eigen::RowVector2d(-0.7, 0.6));
    REQUIRE(result == Eigen::RowVector2d(-1, 0.6));
}

TEST_CASE("fixed corner")
{
    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    AdaptiveTessellation m;
    m.create_mesh(V, F);
    m.mesh_construct_boundaries(V, F, {}, {});
    for (auto v : m.get_vertices()) {
        REQUIRE(m.vertex_attrs[v.vid(m)].fixed);
    }
}

TEST_CASE("boundary parametrization")
{
    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    wmtk::Boundary bnd;
    bnd.construct_boundaries(V, F, {}, {});

    REQUIRE(bnd.num_curves() == 1);

    REQUIRE(bnd.curve_size(0) == 4);

    REQUIRE(bnd.get_t_at_x(0, 0) == 0);
    REQUIRE(bnd.get_t_at_x(0, 1) == 10);
    REQUIRE(bnd.get_t_at_x(0, 2) == 10 + 10 * sqrt(2));
    REQUIRE(bnd.get_t_at_x(0, 3) == 10 + 10 + 10 * sqrt(2));

    double t;

    Eigen::Vector2d test_v0(0, 0);
    auto t0 = bnd.uv_to_t(test_v0).second;
    REQUIRE(t0 == 0.);
    auto v0 = bnd.t_to_uv(0, t0);
    REQUIRE(v0 == test_v0);
    auto ij0 = bnd.uv_to_ij(v0, t);
    assert(t == t0);
    REQUIRE(ij0.first == 0);
    REQUIRE(ij0.second == 0);

    Eigen::Vector2d test_v1(5, 0);
    auto t1 = bnd.uv_to_t(test_v1).second;
    REQUIRE(t1 == 5.);
    auto v1 = bnd.t_to_uv(0, t1);
    REQUIRE(v1 == test_v1);
    auto ij1 = bnd.uv_to_ij(v1, t);
    REQUIRE(t == t1);
    REQUIRE(ij1.first == 0);
    REQUIRE(ij1.second == 0);

    Eigen::Vector2d test_v2(5, 5);
    auto t2 = bnd.uv_to_t(test_v2).second;
    REQUIRE(t2 == 10 + 5. * sqrt(2));
    auto v2 = bnd.t_to_uv(0, t2);
    REQUIRE(v2 == test_v2);
    auto ij2 = bnd.uv_to_ij(v2, t);
    REQUIRE(t == t2);
    REQUIRE(ij2.first == 0);
    REQUIRE(ij2.second == 1);

    Eigen::Vector2d test_v3(0, 0.1);
    auto t3 = bnd.uv_to_t(test_v3).second;
    REQUIRE(t3 == 10 + 10. * sqrt(2) + 9.9);
    auto v3 = bnd.t_to_uv(0, t3);
    REQUIRE((v3 - test_v3).stableNorm() < 1e-8);
    auto ij3 = bnd.uv_to_ij(v3, t);
    REQUIRE(t == t3);
    REQUIRE(ij3.first == 0);
    REQUIRE(ij3.second == 2);
}

TEST_CASE("operations with boundary parameterization")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    AdaptiveTessellation m;
    m.create_mesh(V, F);
    m.set_projection();
    m.mesh_parameters.m_ignore_embedding = true;
    m.mesh_parameters.m_early_stopping_number = 10000;
    m.mesh_parameters.m_boundary.construct_boundaries(V, F, {}, {});
    m.mesh_parameters.m_do_not_output = true;
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        (void)u;
        (void)v;
        return DScalar(1);
    };
    auto displacement_double = [](double u, double v) -> double { return 1; };
    auto displacement_vector = [&displacement_double](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement_double(u, v));
        return p;
    };

    SECTION("smooth")
    {
        m.set_parameters(4, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, true);

        for (auto v : m.get_vertices()) {
            REQUIRE(m.vertex_attrs[v.vid(m)].t >= 0);
            m.vertex_attrs[v.vid(m)].fixed = false;
        }
        REQUIRE(m.mesh_parameters.m_boundary.num_curves() != 0);
        m.smooth_all_vertices();

        for (auto v : m.get_vertices()) {
            auto v_project = m.mesh_parameters.m_get_closest_point(m.vertex_attrs[v.vid(m)].pos);
            REQUIRE((v_project.transpose() - m.vertex_attrs[v.vid(m)].pos).squaredNorm() < 1e-5);
        }
    }

    SECTION("split")
    {
        m.set_parameters(2, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, true);
        m.split_all_edges();

        for (auto e : m.get_edges()) {
            if (m.is_boundary_edge(e)) {
                auto v1_pos = m.mesh_parameters.m_boundary.t_to_uv(0, m.vertex_attrs[e.vid(m)].t);
                auto v2_pos = m.mesh_parameters.m_boundary.t_to_uv(
                    0,
                    m.vertex_attrs[e.switch_vertex(m).vid(m)].t);
                REQUIRE(
                    (m.mesh_parameters.m_get_closest_point(v1_pos) - v1_pos.transpose())
                        .squaredNorm() < 1e-5);
                REQUIRE(
                    (m.mesh_parameters.m_get_closest_point(v2_pos) - v2_pos.transpose())
                        .squaredNorm() < 1e-5);
            }
        }
    }

    SECTION("collapse")
    {
        m.set_parameters(10, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, true);
        wmtk::logger().info(m.mesh_parameters.m_quality_threshold);
        m.collapse_all_edges();

        for (auto e : m.get_edges()) {
            if (m.is_boundary_edge(e)) {
                auto v1_pos = m.mesh_parameters.m_boundary.t_to_uv(0, m.vertex_attrs[e.vid(m)].t);
                auto v2_pos = m.mesh_parameters.m_boundary.t_to_uv(
                    0,
                    m.vertex_attrs[e.switch_vertex(m).vid(m)].t);
                REQUIRE(
                    (m.mesh_parameters.m_get_closest_point(v1_pos) - v1_pos.transpose())
                        .squaredNorm() < 1e-5);
                REQUIRE(
                    (m.mesh_parameters.m_get_closest_point(v2_pos) - v2_pos.transpose())
                        .squaredNorm() < 1e-5);
            }
        }
    }
}

TEST_CASE("autodiff vs finitediff")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    AdaptiveTessellation m;
    m.create_mesh(V, F);
    m.mesh_construct_boundaries(V, F, {}, {});

    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        return DScalar(10 * u);
    };
    auto displacement_vector = [](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, 10 * u);
        return p;
    };
    m.set_parameters(4, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, true);

    for (auto v : m.get_vertices()) {
        REQUIRE(m.vertex_attrs[v.vid(m)].t >= 0);
        m.vertex_attrs[v.vid(m)].fixed = false;
    }
    m.smooth_all_vertices();

    Eigen::VectorXd v_flat;
    m.flatten_dofs(v_flat);
    Eigen::VectorXd finitediff_grad = Eigen::VectorXd::Zero(m.get_vertices().size() * 2);

    std::function<double(const Eigen::VectorXd&)> f =
        [&m](const Eigen::VectorXd& v_flat) -> double { return m.get_mesh_energy(v_flat); };
    fd::finite_gradient(v_flat, f, finitediff_grad, fd::SECOND, 1e-2);

    for (auto v : m.get_vertices()) {
        auto autodiff = m.get_one_ring_energy(v);
        Eigen::Vector2d fd_grad =
            Eigen::Vector2d(finitediff_grad[v.vid(m) * 2], finitediff_grad[v.vid(m) * 2 + 1]);
        wmtk::logger().info(
            "boundary {} autodiff {} finitediff {}, {}",
            m.is_boundary_vertex(v),
            autodiff.second,
            finitediff_grad[v.vid(m) * 2],
            finitediff_grad[v.vid(m) * 2 + 1]);
    }
}

// TODO: Try out sin(x) with periodic boundary cond + autodiff + gradient

TEST_CASE("paired split")
{
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
    wmtk::TriMesh::Tuple primary_edge3 = wmtk::TriMesh::Tuple(0, 1, 0, m);
    REQUIRE(!m.edge_attrs[primary_edge3.eid(m)].curve_id.has_value());
    REQUIRE(!m.is_seam_edge(primary_edge3));
    REQUIRE(!m.is_boundary_edge(primary_edge3));
    AdaptiveTessellationPairedSplitEdgeOperation op2;
    op2(m, primary_edge3);
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
    wmtk::TriMesh::Tuple primary_edge4 = wmtk::TriMesh::Tuple(3, 2, 1, m);
    REQUIRE(m.edge_attrs[primary_edge4.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge4.eid(m)].curve_id.value() == 2);
    REQUIRE(m.is_boundary_edge(primary_edge4));
    REQUIRE(!m.is_seam_edge(primary_edge4));
    AdaptiveTessellationPairedSplitEdgeOperation op3;
    op3(m, primary_edge4);
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
}

TEST_CASE("paired collapse")
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
    //           \   |pe5|      |      /
    //            \ f0\f4|      | f3  /
    //             \   | |      |    /
    //              \(1)\|      |(2)/
    //                2           5

    wmtk::TriMesh::Tuple primary_edge6 = wmtk::TriMesh::Tuple(6, 1, 4, m);
    REQUIRE(m.is_seam_vertex(primary_edge6));
    REQUIRE(!m.is_boundary_edge(primary_edge6));
    REQUIRE(!m.is_seam_edge(primary_edge6));
    REQUIRE(!m.edge_attrs[primary_edge6.eid(m)].curve_id.has_value());
    REQUIRE(primary_edge6.is_valid(m));
    AdaptiveTessellationPairedCollapseEdgeOperation op4;
    op4(m, primary_edge6);
    // acsii art diamond
    //                1    4
    //              /(2) ||(1)\ 
    //             /     ||    \ 9
    //            /f2    || f6 /\     
    //           /       ||   /  \    
    //          /        ||  /f1  \   
    //        0(0)-----10||7----(0)3
    //          \       |||       /
    //           \      |||      /
    //            \ f0 \||| f3  /
    //             \     ||    /
    //              \(1) ||(2)/
    //                2     5
    const auto& primary_edge6_opt = op4.collapse_edge.get_return_tuple_opt();
    REQUIRE(primary_edge6_opt.has_value());
    const auto& primary_edge6_ret = primary_edge6_opt.value();
    REQUIRE(primary_edge6_ret.is_valid(m));
    REQUIRE(primary_edge6_ret.vid(m) == 10);
    REQUIRE(m.vert_capacity() == 9);
    REQUIRE(m.is_seam_edge(primary_edge6_ret));
    REQUIRE(m.edge_attrs[primary_edge6_ret.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge6_ret.eid(m)].curve_id.value() == 1);
    REQUIRE(m.get_oriented_mirror_edge(primary_edge6_ret).is_valid(m));
    REQUIRE(m.get_oriented_mirror_edge(primary_edge6_ret).fid(m) == 3);
    REQUIRE(m.get_oriented_mirror_edge(primary_edge6_ret).vid(m) == 5);
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
    wmtk::TriMesh::Tuple primary_edge7 = wmtk::TriMesh::Tuple(3, 2, 1, m);
    REQUIRE(primary_edge7.switch_vertex(m).vid(m) == 9);
    REQUIRE(!m.is_seam_vertex(primary_edge7));
    REQUIRE(m.is_boundary_edge(primary_edge7));
    REQUIRE(!m.is_seam_edge(primary_edge7));
    REQUIRE(m.edge_attrs[primary_edge7.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge7.eid(m)].curve_id.value() == 2);

    AdaptiveTessellationPairedCollapseEdgeOperation op5;
    op5(m, primary_edge7);
    // acsii art diamond
    //                1    4
    //              /(2) ||(1)\ 
    //             /     ||    \ 
    //            /f2    || f6  \     
    //           /       ||      \    
    //          /        ||       \   
    //        0(0)-----10||7----(0)11
    //          \        || <---  /
    //           \       ||      /
    //            \ f0   || f3  /
    //             \     ||    /
    //              \(1) ||(2)/
    //                2     5
    const auto& primary_edge7_opt = op5.collapse_edge.get_return_tuple_opt();
    REQUIRE(primary_edge7_opt.has_value());
    const auto& primary_edge7_ret = primary_edge7_opt.value();
    REQUIRE(primary_edge7_ret.is_valid(m));
    REQUIRE(primary_edge7_ret.vid(m) == 11);
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
    REQUIRE(!m.edge_attrs[primary_edge7_ret.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge7_ret.switch_edge(m).eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge7_ret.switch_edge(m).eid(m)].curve_id.value() == 2);

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
    wmtk::TriMesh::Tuple primary_edge8 = wmtk::TriMesh::Tuple(7, 0, 3, m);
    REQUIRE(primary_edge8.switch_vertex(m).vid(m) == 5);
    REQUIRE(m.is_seam_vertex(primary_edge8));
    REQUIRE(m.is_seam_edge(primary_edge8));
    REQUIRE(m.edge_attrs[primary_edge8.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge8.eid(m)].curve_id.value() == 0);

    AdaptiveTessellationPairedCollapseEdgeOperation op6;
    op6(m, primary_edge8);

    // acsii art diamond
    //                1    4
    //              /(2) ||(1)\ 
    //             /     ||    \ 9
    //            /      ||     \     
    //           /       ||      \    
    //          /    f2  ||  f3   \   
    //        0(0)       ||      (0)11
    //          \|\      ||     /|/
    //           \ \pe9  || pe8/ /
    //            \ \    ||   / /
    //             \     ||    /
    //              \(1) ||(2)/
    //                13    12

    const auto& primary_edge8_opt = op6.collapse_edge.get_return_tuple_opt();
    const auto& primary_edge9_opt = op6.collapse_mirror_edge.get_return_tuple_opt();
    REQUIRE(primary_edge8_opt.has_value());
    REQUIRE(primary_edge9_opt.has_value());
    const auto& primary_edge8_ret = primary_edge8_opt.value();
    const auto& primary_edge9_ret = primary_edge9_opt.value();
    REQUIRE(primary_edge8_ret.vid(m) == 12);
    REQUIRE(primary_edge9_ret.vid(m) == 13);
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
    REQUIRE(valid_faces_cnt == 2);
    REQUIRE(!m.is_seam_edge(primary_edge8_ret));
    REQUIRE(!m.is_seam_edge(primary_edge9_ret));
    REQUIRE(m.is_boundary_edge(primary_edge8_ret));
    REQUIRE(m.is_boundary_edge(primary_edge9_ret));
    REQUIRE(m.edge_attrs[primary_edge8_ret.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge8_ret.eid(m)].curve_id.value() == 2);
    REQUIRE(m.edge_attrs[primary_edge9_ret.eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[primary_edge9_ret.eid(m)].curve_id.value() == 3);
    REQUIRE(m.is_seam_edge(primary_edge8_ret.switch_edge(m)));
    REQUIRE(m.is_seam_vertex(primary_edge8_ret));
    REQUIRE(m.is_seam_edge(primary_edge9_ret.switch_edge(m)));
    REQUIRE(m.is_seam_vertex(primary_edge9_ret));
    REQUIRE(m.get_oriented_mirror_edge(primary_edge8_ret.switch_edge(m)).vid(m) == 1);

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
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 3);
                REQUIRE(m.get_oriented_mirror_edge(e).vid(m) == 4);
                break;
            case 1:
                REQUIRE(m.get_oriented_mirror_edge(e).fid(m) == 3);
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
    wmtk::TriMesh::Tuple primary_edge5 = wmtk::TriMesh::Tuple(7, 1, 6, m);
    REQUIRE(primary_edge5.is_valid(m));
    REQUIRE(primary_edge5.vid(m) == 7);
    REQUIRE(primary_edge5.switch_vertex(m).vid(m) == 9);
    AdaptiveTessellationSwapEdgeOperation op4;
    // this operation should fail because of it would generate colinear traingles
    op4(m, primary_edge5);
    std::vector<TriMeshTuple> op4_modified_tuples = op4.modified_tuples(m);
    REQUIRE(!op4);
    REQUIRE(op4_modified_tuples.size() == 0);

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

    wmtk::TriMesh::Tuple primary_edge6 = wmtk::TriMesh::Tuple(7, 2, 3, m);
    REQUIRE(primary_edge6.is_valid(m));
    REQUIRE(primary_edge6.vid(m) == 7);
    REQUIRE(primary_edge6.switch_vertex(m).vid(m) == 3);
    AdaptiveTessellationSwapEdgeOperation op5;
    // this operation should fail because of it would generate colinear traingles
    op5(m, primary_edge6);
    std::vector<TriMeshTuple> op5_modified_tuples = op5.modified_tuples(m);

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

    REQUIRE(op5_modified_tuples.size() == 2);
    const wmtk::TriMesh::Tuple op5_ret = op5.get_return_tuple_opt().value();
    REQUIRE(op5_ret.is_valid(m));
    REQUIRE(op5_ret.vid(m) == 5);
    // edge 3-5
    REQUIRE(m.is_boundary_edge(op5_ret.switch_edge(m)));
    REQUIRE(m.edge_attrs[op5_ret.switch_edge(m).eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[op5_ret.switch_edge(m).eid(m)].curve_id.value() == 2);
    // edge 5-7
    REQUIRE(m.is_seam_edge(op5_modified_tuples[1].switch_edge(m)));
    REQUIRE(m.edge_attrs[op5_modified_tuples[1].switch_edge(m).eid(m)].curve_id.has_value());
    REQUIRE(m.edge_attrs[op5_modified_tuples[1].switch_edge(m).eid(m)].curve_id.value() == 0);
    REQUIRE(m.get_oriented_mirror_edge(op5_modified_tuples[1].switch_edge(m)).vid(m) == 6);
    REQUIRE(m.get_oriented_mirror_edge(op5_modified_tuples[1].switch_edge(m)).fid(m) == 4);
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
    wmtk::TriMesh::Tuple primary_edge7 = wmtk::TriMesh::Tuple(5, 0, 3, m);
    REQUIRE(primary_edge7.is_valid(m));
    REQUIRE(primary_edge7.vid(m) == 5);
    REQUIRE(primary_edge7.switch_vertex(m).vid(m) == 7);
    REQUIRE(m.is_seam_edge(primary_edge7));
    AdaptiveTessellationSwapEdgeOperation op6;
    op6(m, primary_edge7);
    std::vector<TriMeshTuple> op6_modified_tuples = op6.modified_tuples(m);
    REQUIRE(!op6);
    REQUIRE(op6_modified_tuples.size() == 0);

    ////////// ======= boundary edge swap
    /// should be rejected. don't swap boundary
    wmtk::TriMesh::Tuple primary_edge8 = wmtk::TriMesh::Tuple(2, 2, 0, m);
    REQUIRE(primary_edge8.is_valid(m));
    REQUIRE(primary_edge8.vid(m) == 2);
    REQUIRE(primary_edge8.switch_vertex(m).vid(m) == 0);
    REQUIRE(m.is_boundary_edge(primary_edge8));
    AdaptiveTessellationSwapEdgeOperation op7;
    op7(m, primary_edge8);
    REQUIRE(!op7);
    std::vector<TriMeshTuple> op7_modified_tuples = op7.modified_tuples(m);
    REQUIRE(op7_modified_tuples.size() == 0);
}

TEST_CASE("test mirror edge setup")
{
    AdaptiveTessellation m;
    Eigen::MatrixXd UV;
    Eigen::MatrixXi F;
    Eigen::MatrixXd V3d;
    Eigen::MatrixXi F3d;
    Eigen::MatrixXd CN, FN;
    std::string input_mesh_path = WMTK_DATA_DIR "/hemisphere.obj";
    igl::readOBJ(input_mesh_path, V3d, UV, CN, F3d, F, FN);
    std::string displaced_positions = WMTK_DATA_DIR "/images/hemisphere_512_displaced.exr";
    m.mesh_preprocessing(input_mesh_path, displaced_positions);
    for (const auto& f : m.get_faces()) {
        size_t fi = f.fid(m);
        for (auto i = 0; i < 3; ++i) {
            auto mirror_edge = m.face_attrs[fi].mirror_edges[i];
            wmtk::TriMesh::Tuple tup = m.tuple_from_edge(fi, i);
            auto lv1 = (i + 1) % 3;
            auto lv2 = (i + 2) % 3;
            REQUIRE(3 - lv1 - lv2 == i);
            REQUIRE(tup.vid(m) == F(fi, lv1));
            if (mirror_edge.has_value()) {
                auto fj = mirror_edge.value().fid(m);
                auto mirror_vid1 = mirror_edge.value().vid(m);
                auto mirror_vid2 = mirror_edge.value().switch_vertex(m).vid(m);
                auto mirror_lv1 = (mirror_edge.value().local_eid(m) + 1) % 3;
                auto mirror_lv2 = (mirror_edge.value().local_eid(m) + 2) % 3;
                auto original_tup =
                    m.face_attrs[fj].mirror_edges[mirror_edge.value().local_eid(m)].value();
                REQUIRE(
                    (original_tup.vid(m) == tup.vid(m) ||
                     original_tup.switch_vertex(m).vid(m) == tup.vid(m)));

                REQUIRE(F(fi, lv1) != F(fj, mirror_lv1));
                REQUIRE(F(fi, lv2) != F(fj, mirror_lv2));
            }
        }
    }
}

TEST_CASE("get mirror")
{
    AdaptiveTessellation m;
    Eigen::MatrixXd UV;
    Eigen::MatrixXi F;
    Eigen::MatrixXd V3d;
    Eigen::MatrixXi F3d;
    Eigen::MatrixXd CN, FN;
    std::string input_mesh_path = WMTK_DATA_DIR "/hemisphere.obj";
    igl::readOBJ(input_mesh_path, V3d, UV, CN, F3d, F, FN);
    std::string displaced_positions = WMTK_DATA_DIR "/images/hemisphere_512_displaced.exr";
    m.mesh_preprocessing(input_mesh_path, displaced_positions);
    for (const auto& f : m.get_faces()) {
        size_t fi = f.fid(m);
        for (auto i = 0; i < 3; ++i) {
            //          F(fi, lv1)    F(fj, lv1)
            //          lv1\ \         -----------
            //          /   \ \         \        /
            //         /  fi \ \|     |\ \  fj  /
            //        /       \         \ \    /
            //       /_____lv2_\         \ \  /
            //     F(fi,i)    F(fi,lv2)     F(fj, lv2)

            auto lv1 = (i + 1) % 3;
            auto lv2 = (i + 2) % 3;
            REQUIRE(3 - lv1 - lv2 == i);
            auto mirror_edge = m.face_attrs[fi].mirror_edges[i];
            wmtk::TriMesh::Tuple tup = wmtk::TriMesh::Tuple(F(fi, lv1), i, fi, m);
            REQUIRE(tup.vid(m) == F(fi, lv1));
            if (mirror_edge.has_value()) {
                auto fj = mirror_edge.value().fid(m);
                int lvj1 = -1;
                int lvj2 = -1;
                for (int i = 0; i < 3; i++) {
                    if (F3d(fj, i) == F3d(fi, lv1)) lvj1 = i;
                    if (F3d(fj, i) == F3d(fi, lv2)) lvj2 = i;
                }

                REQUIRE((F(fi, lv1) != F(fj, lvj1) || F(fi, lv2) != F(fj, lvj2)));
                REQUIRE(F3d(fi, lv1) == F3d(fj, lvj1));
                REQUIRE(F3d(fi, lv2) == F3d(fj, lvj2));

                REQUIRE(m.is_seam_edge(mirror_edge.value()));
                REQUIRE(m.is_seam_edge(tup));
                auto get_back_tup =
                    m.face_attrs[fj].mirror_edges[mirror_edge.value().local_eid(m)].value();
                auto mirror_edge_with_getter = m.get_oriented_mirror_edge(tup);
                if (lvj2 != -1) REQUIRE(mirror_edge_with_getter.vid(m) == F(fj, lvj2));
                REQUIRE(mirror_edge_with_getter.fid(m) == fj);
                if (tup.is_ccw(m)) {
                    REQUIRE(tup.vid(m) == get_back_tup.vid(m));
                    REQUIRE(get_back_tup.vid(m) == F(fi, lv1));
                    REQUIRE(mirror_edge_with_getter.is_ccw(m));
                } else {
                    REQUIRE(tup.vid(m) == get_back_tup.switch_vertex(m).vid(m));
                    REQUIRE(get_back_tup.vid(m) == F(fi, lv2));
                    REQUIRE(!mirror_edge_with_getter.is_ccw(m));
                }
                auto mirror_vertex_with_getter = m.get_mirror_vertex(tup);
                REQUIRE(mirror_vertex_with_getter.vid(m) == F(fj, lvj1));
                REQUIRE(
                    m.get_mirror_vertex(mirror_edge_with_getter).vid(m) ==
                    tup.switch_vertex(m).vid(m));
                REQUIRE(m.get_mirror_vertex(mirror_edge_with_getter).vid(m) == F(fi, lv2));
            }
        }
    }
}

// all vertices that have coloring that have more than 2 vertices should be fixed
TEST_CASE("test curve fixed get_all_mirror_vids")
{
    AdaptiveTessellation m;
    std::string input_mesh_path = WMTK_DATA_DIR "/hemisphere.obj";
    std::string displaced_positions = WMTK_DATA_DIR "/images/hemisphere_512_displaced.exr";
    m.mesh_preprocessing(input_mesh_path, displaced_positions);
    for (auto i = 0; i < m.vert_capacity(); ++i) {
        if (m.color_to_uv_indices[m.uv_index_to_color[i]].size() > 2) {
            REQUIRE(m.vertex_attrs[i].fixed);
        }
    }

    auto uv_last =
        m.mesh_parameters.m_boundary.t_to_uv(9, m.mesh_parameters.m_boundary.upper_bound(9));
    REQUIRE(m.mesh_parameters.m_boundary.curve_size(9) == 2);
    REQUIRE((uv_last - m.mesh_parameters.m_boundary.get_position_at_x(9, 1)).squaredNorm() < 1e-5);

    // test for get_all_mirror_vids
    for (auto i = 0; i < m.vert_capacity(); ++i) {
        if (m.color_to_uv_indices[m.uv_index_to_color[i]].size() > 2) {
            REQUIRE(m.vertex_attrs[i].fixed);
            REQUIRE(m.tuple_from_vertex(i).is_valid(m));
            auto vector_vids = m.get_all_mirror_vids(m.tuple_from_vertex(i));
            for (auto vid : vector_vids) {
                REQUIRE(
                    std::find(
                        m.color_to_uv_indices[m.uv_index_to_color[i]].begin(),
                        m.color_to_uv_indices[m.uv_index_to_color[i]].end(),
                        vid) != m.color_to_uv_indices[m.uv_index_to_color[i]].end());
            }
        }
    }
}

TEST_CASE("uv-index and coloring test")
{
    AdaptiveTessellation m;
    Eigen::MatrixXd UV;
    Eigen::MatrixXi F;
    Eigen::MatrixXd V3d;
    Eigen::MatrixXi F3d;
    Eigen::MatrixXd CN, FN;
    std::string input_mesh_path = WMTK_DATA_DIR "/hemisphere.obj";
    igl::readOBJ(input_mesh_path, V3d, UV, CN, F3d, F, FN);
    std::string displaced_positions = WMTK_DATA_DIR "/images/hemisphere_512_displaced.exr";
    m.mesh_preprocessing(input_mesh_path, displaced_positions);

    for (auto& v : m.get_vertices()) {
        // iterate through vertices
        int color = m.uv_index_to_color[v.vid(m)];
        if (m.is_seam_vertex(v)) {
            for (auto& e : m.get_one_ring_edges_for_vertex(v)) {
                if (m.is_seam_edge(e)) {
                    REQUIRE(e.switch_vertex(m).vid(m) == v.vid(m));
                    wmtk::TriMesh::Tuple mirror_vertex_with_getter =
                        m.get_mirror_vertex(e.switch_vertex(m));
                    REQUIRE(m.uv_index_to_color[mirror_vertex_with_getter.vid(m)] == color);
                    REQUIRE(
                        std::find(
                            m.color_to_uv_indices[color].begin(),
                            m.color_to_uv_indices[color].end(),
                            v.vid(m)) != m.color_to_uv_indices[color].end());
                    REQUIRE(
                        std::find(
                            m.color_to_uv_indices[color].begin(),
                            m.color_to_uv_indices[color].end(),
                            mirror_vertex_with_getter.vid(m)) !=
                        m.color_to_uv_indices[color].end());
                }
            }
        }
    }
}

// TODO special case for link condition in seamed mesh. a tube with a seam edge in
// the middle

TEST_CASE("edge curve-id assignment")
{
    AdaptiveTessellation m;
    std::filesystem::path input_mesh_path = WMTK_DATA_DIR "/hemisphere.obj";
    std::string displaced_positions = WMTK_DATA_DIR "/images/hemisphere_512_displaced.exr";
    m.mesh_preprocessing(input_mesh_path.string(), displaced_positions);
    for (auto& e : m.get_edges()) {
        if (m.is_boundary_edge(e)) {
            REQUIRE(m.edge_attrs[e.eid(m)].curve_id.has_value());
        }
    }
    for (int curve_id = 0; curve_id < m.mesh_parameters.m_boundary.num_curves(); curve_id++) {
        for (int positionx = 0; positionx < m.mesh_parameters.m_boundary.curve_size(curve_id) - 1;
             positionx++) {
            auto uv1 = m.mesh_parameters.m_boundary.get_position_at_x(curve_id, positionx);
            auto uv2 = m.mesh_parameters.m_boundary.get_position_at_x(curve_id, positionx + 1);
            double dist_first = std::numeric_limits<double>::infinity();
            double dist_last = std::numeric_limits<double>::infinity();
            size_t vid_1 = -1;
            size_t vid_2 = -1;
            for (auto& v : m.get_vertices()) {
                if ((m.vertex_attrs[v.vid(m)].pos - uv1).squaredNorm() < dist_first) {
                    dist_first = (m.vertex_attrs[v.vid(m)].pos - uv1).squaredNorm();
                    vid_1 = v.vid(m);
                }
                if ((m.vertex_attrs[v.vid(m)].pos - uv2).squaredNorm() < dist_last) {
                    dist_last = (m.vertex_attrs[v.vid(m)].pos - uv2).squaredNorm();
                    vid_2 = v.vid(m);
                }
            }
            assert(dist_first < 1e-8);
            assert(dist_last < 1e-8);
            auto e = m.tuple_from_vertex(vid_1);
            for (auto& one_ring_e : m.get_one_ring_edges_for_vertex(e)) {
                if (one_ring_e.vid(m) == vid_2) {
                    e = one_ring_e;
                    break;
                }
            }
            REQUIRE(m.edge_attrs[e.eid(m)].curve_id.has_value());
            REQUIRE(m.edge_attrs[e.eid(m)].curve_id.value() == curve_id);
        }
    }
}

TEST_CASE("quickrun")
{
    // Loading the input 2d mesh
    AdaptiveTessellation m;
    m.mesh_preprocessing("swap_0006.obj", "/home/yunfan/seamPyramid_displaced.exr");
    Image image;
    image.load(
        "/home/yunfan/seamPyramid_height_10.exr",
        WrappingMode::MIRROR_REPEAT,
        WrappingMode::MIRROR_REPEAT);

    m.mesh_parameters.m_position_normal_paths = {"/home/yunfan/seamPyramid_position.exr",
                                                 "/home/yunfan/seamPyramid_normal_smooth.exr"};
    REQUIRE(m.check_mesh_connectivity_validity());
    m.mesh_parameters.m_early_stopping_number = 100;

    m.set_parameters(
        0.00001,
        0.4,
        image,
        WrappingMode::MIRROR_REPEAT,
        SAMPLING_MODE::BICUBIC,
        DISPLACEMENT_MODE::MESH_3D,
        adaptive_tessellation::ENERGY_TYPE::AREA_QUADRATURE,
        adaptive_tessellation::EDGE_LEN_TYPE::AREA_ACCURACY,
        1);
    // m.split_all_edges();
    // m.write_obj_displaced("split_result.obj");
    m.swap_all_edges();
    m.write_obj_displaced("swap_result.obj");
    m.smooth_all_vertices();
    m.write_obj_displaced("smooth_result.obj");
    m.write_obj("smooth_result_2d.obj");
}

TEST_CASE("check curveid consistency after split")
{
    // logger().set_level(spdlog::level::trace);
    // Loading the input 2d mesh
    AdaptiveTessellation m;
    m.mesh_preprocessing("/home/yunfan/seamPyramid.obj", "/home/yunfan/seamPyramid_displaced.exr");
    Image image;
    image.load(
        "/home/yunfan/seamPyramid_height_10.exr",
        WrappingMode::MIRROR_REPEAT,
        WrappingMode::MIRROR_REPEAT);

    m.mesh_parameters.m_position_normal_paths = {"/home/yunfan/seamPyramid_position.exr",
                                                 "/home/yunfan/seamPyramid_normal_smooth.exr"};
    assert(m.check_mesh_connectivity_validity());
    // stop after 100 iterations
    m.mesh_parameters.m_early_stopping_number = 100;
    m.set_parameters(
        0.00001,
        0.4,
        image,
        WrappingMode::MIRROR_REPEAT,
        SAMPLING_MODE::BICUBIC,
        DISPLACEMENT_MODE::MESH_3D,
        adaptive_tessellation::ENERGY_TYPE::AREA_QUADRATURE,
        adaptive_tessellation::EDGE_LEN_TYPE::AREA_ACCURACY,
        1);
    m.split_all_edges();
    m.write_obj_displaced("split_result.obj");
    m.write_obj_only_texture_coords("split_result_2d.obj");
    // check curve-id after split per edge
    for (auto& e : m.get_edges()) {
        if (m.is_boundary_edge(e)) {
            wmtk::logger().info(e.info());
            REQUIRE(m.edge_attrs[e.eid(m)].curve_id.has_value());
            // find the mid-point uv of the edge
            auto uv =
                (m.vertex_attrs[e.vid(m)].pos + m.vertex_attrs[e.switch_vertex(m).vid(m)].pos) / 2.;
            // find the curve id of the mid-point uv
            int curve_id = -1;
            double t = 0.;
            std::tie(curve_id, t) = m.mesh_parameters.m_boundary.uv_to_t(uv);
            REQUIRE(m.edge_attrs[e.eid(m)].curve_id.value() == curve_id);
        } else {
            REQUIRE(!m.edge_attrs[e.eid(m)].curve_id.has_value());
        }
    }
}

TEST_CASE("logging")
{
    AdaptiveTessellation m;

    m.mesh_parameters.log({{"logging info", {"your info should be in runtime.log"}}});
}

TEST_CASE("mirror vertex t_to_uv")
{
    AdaptiveTessellation m;
    m.mesh_preprocessing(
        WMTK_DATA_DIR "/hemisphere_splited.obj",
        WMTK_DATA_DIR "/images/hemisphere_512_displaced.exr");
    wmtk::Boundary bd_map = m.mesh_parameters.m_boundary;
    for (auto& e : m.get_edges()) {
        if (m.is_seam_edge(e)) {
            // first vertex

            if (m.vertex_attrs[e.vid(m)].fixed) continue;
            wmtk::TriMesh::Tuple mirror_v = m.get_mirror_vertex(e);
            // t and mirror_v.t should be the same
            REQUIRE_THAT(
                m.vertex_attrs[mirror_v.vid(m)].t,
                Catch::Matchers::WithinAbs(m.vertex_attrs[e.vid(m)].t, (float)1e-7));
            REQUIRE(m.get_mirror_vertex(mirror_v).vid(m) == e.vid(m));
            REQUIRE(m.edge_attrs[mirror_v.eid(m)].curve_id.has_value());
            REQUIRE(m.edge_attrs[e.eid(m)].curve_id.has_value());

            // the uv pos for mirror vertex should be the same as result of t_to_uv
            if (!(m.vertex_attrs[mirror_v.vid(m)].pos -
                  bd_map.t_to_uv(
                      m.edge_attrs[mirror_v.eid(m)].curve_id.value(),
                      m.vertex_attrs[mirror_v.vid(m)].t))
                     .squaredNorm() < 1e-5) {
                wmtk::logger().error("mirror vertex uv pos not match");
                wmtk::logger().info(
                    "curve id {}, t {}",
                    m.edge_attrs[mirror_v.eid(m)].curve_id.value(),
                    m.vertex_attrs[mirror_v.vid(m)].t);
                wmtk::logger().info(
                    "t to uv {} uv pos {}",
                    bd_map.t_to_uv(
                        m.edge_attrs[mirror_v.eid(m)].curve_id.value(),
                        m.vertex_attrs[mirror_v.vid(m)].t),
                    m.vertex_attrs[mirror_v.vid(m)].pos);
                REQUIRE(false);
            }
            // second vertex
            if (m.vertex_attrs[e.switch_vertex(m).vid(m)].fixed) continue;
            wmtk::TriMesh::Tuple mirror_v2 = m.get_mirror_vertex(e.switch_vertex(m));
            // t and mirror_v.t should be the same
            REQUIRE_THAT(
                m.vertex_attrs[mirror_v2.vid(m)].t,
                Catch::Matchers::WithinAbs(
                    m.vertex_attrs[e.switch_vertex(m).vid(m)].t,
                    (float)1e-7));

            REQUIRE(m.get_mirror_vertex(mirror_v2).vid(m) == e.switch_vertex(m).vid(m));
            REQUIRE(m.edge_attrs[mirror_v2.eid(m)].curve_id.has_value());
            REQUIRE(m.edge_attrs[e.eid(m)].curve_id.has_value());
            // the uv pos for mirror vertex should be the same as result of t_to_uv
            if (!(m.vertex_attrs[mirror_v2.vid(m)].pos -
                  bd_map.t_to_uv(
                      m.edge_attrs[mirror_v2.eid(m)].curve_id.value(),
                      m.vertex_attrs[mirror_v2.vid(m)].t))
                     .squaredNorm() < 1e-5) {
                wmtk::logger().error("mirror vertex uv pos not match");
                wmtk::logger().info(
                    "curve id {}, t {}",
                    m.edge_attrs[mirror_v2.eid(m)].curve_id.value(),
                    m.vertex_attrs[mirror_v2.vid(m)].t);
                wmtk::logger().info(
                    "t to uv {} uv pos {}",
                    bd_map.t_to_uv(
                        m.edge_attrs[mirror_v2.eid(m)].curve_id.value(),
                        m.vertex_attrs[mirror_v2.vid(m)].t),
                    m.vertex_attrs[mirror_v2.vid(m)].pos);
                REQUIRE(false);
            }
        }
    }
}
