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
#include "Split.h"

using namespace wmtk;
using namespace lagrange;
using namespace adaptive_tessellation;

template <class T>
using RowMatrix2 = Eigen::Matrix<T, Eigen::Dynamic, 2, Eigen::RowMajor>;
using Index = uint64_t;
using Scalar = double;

TEST_CASE("AABB")
{
    const std::string root(WMT_DATA_DIR);
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
    SECTION("diamond")
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
        m.face_attrs[0].mirror_edges[0] =
            std::make_optional<wmtk::TriMesh::Tuple>(wmtk::TriMesh::Tuple(4, 2, 1, m));
        m.face_attrs[1].mirror_edges[2] =
            std::make_optional<wmtk::TriMesh::Tuple>(wmtk::TriMesh::Tuple(2, 0, 0, m));
        auto tup = wmtk::TriMesh::Tuple(2, 0, 0, m);
        REQUIRE(tup.vid(m) == 2);
        AdaptiveTessellationPairedSplitEdgeOperation op;
        op(m, tup);
        REQUIRE(m.vert_capacity() == 8);
        REQUIRE(m.tri_capacity() == 4);
        // checking for first face
        REQUIRE(m.face_attrs[0].mirror_edges[0].has_value());
        REQUIRE(m.face_attrs[0].mirror_edges[0].value().fid(m) == 1);
        REQUIRE(m.face_attrs[0].mirror_edges[0].value().vid(m) == 4);
        // checking for first face mirroring face
        REQUIRE(m.face_attrs[1].mirror_edges[2].has_value());
        REQUIRE(m.face_attrs[1].mirror_edges[2].value().fid(m) == 0);
        REQUIRE(m.face_attrs[1].mirror_edges[2].value().vid(m) == 2);
        // checking for second face
        REQUIRE(m.face_attrs[2].mirror_edges[0].has_value());
        REQUIRE(m.face_attrs[2].mirror_edges[0].value().fid(m) == 3);
        REQUIRE(m.face_attrs[2].mirror_edges[0].value().vid(m) == 3);
        // checking for second face mirroring face
        REQUIRE(m.face_attrs[3].mirror_edges[2].has_value());
        REQUIRE(m.face_attrs[3].mirror_edges[2].value().fid(m) == 2);
        REQUIRE(m.face_attrs[3].mirror_edges[2].value().vid(m) == 1);
    }
    SECTION("diamond backward edge")
    // to test edge in operation that of opposite direction
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
        m.face_attrs[0].mirror_edges[0] =
            std::make_optional<wmtk::TriMesh::Tuple>(wmtk::TriMesh::Tuple(4, 2, 1, m));
        m.face_attrs[1].mirror_edges[2] =
            std::make_optional<wmtk::TriMesh::Tuple>(wmtk::TriMesh::Tuple(2, 0, 0, m));
        auto tup = wmtk::TriMesh::Tuple(2, 0, 0, m);
        tup = tup.switch_vertex(m);
        REQUIRE(tup.vid(m) == 1);
        AdaptiveTessellationPairedSplitEdgeOperation op;
        op(m, tup);
        REQUIRE(m.vert_capacity() == 8);
        REQUIRE(m.tri_capacity() == 4);
        // checking for first face
        REQUIRE(m.face_attrs[0].mirror_edges[0].has_value());
        REQUIRE(m.face_attrs[0].mirror_edges[0].value().fid(m) == 1);
        REQUIRE(m.face_attrs[0].mirror_edges[0].value().vid(m) == 4);
        // checking for first face mirroring face
        REQUIRE(m.face_attrs[1].mirror_edges[2].has_value());
        REQUIRE(m.face_attrs[1].mirror_edges[2].value().fid(m) == 0);
        REQUIRE(m.face_attrs[1].mirror_edges[2].value().vid(m) == 2);
        // checking for second face
        REQUIRE(m.face_attrs[2].mirror_edges[0].has_value());
        REQUIRE(m.face_attrs[2].mirror_edges[0].value().fid(m) == 3);
        REQUIRE(m.face_attrs[2].mirror_edges[0].value().vid(m) == 3);
        // checking for second face mirroring face
        REQUIRE(m.face_attrs[3].mirror_edges[2].has_value());
        REQUIRE(m.face_attrs[3].mirror_edges[2].value().fid(m) == 2);
        REQUIRE(m.face_attrs[3].mirror_edges[2].value().vid(m) == 1);
    }
    SECTION("isosceles triangles")
    {
        Eigen::MatrixXd V(4, 2);
        Eigen::MatrixXi F(2, 3);
        V.row(0) << -1., 0.;
        V.row(1) << 0., 1.;
        V.row(2) << 0., 0;
        V.row(3) << 1., 0;
        F.row(0) << 0, 2, 1;
        F.row(1) << 1, 2, 3;
        AdaptiveTessellation m;
        m.create_mesh_debug(V, F);
        REQUIRE(m.check_mesh_connectivity_validity());
        m.face_attrs[0].mirror_edges[1] =
            std::make_optional<wmtk::TriMesh::Tuple>(wmtk::TriMesh::Tuple(3, 1, 1, m));
        m.face_attrs[1].mirror_edges[1] =
            std::make_optional<wmtk::TriMesh::Tuple>(wmtk::TriMesh::Tuple(0, 1, 0, m));
        auto tup = m.tuple_from_edge(0, 1);
        REQUIRE(tup.is_valid(m));
        REQUIRE(tup.vid(m) == 1);

        AdaptiveTessellationPairedSplitEdgeOperation op;
        op(m, tup);
        REQUIRE(m.vert_capacity() == 6);
        REQUIRE(m.tri_capacity() == 4);
        // checking for first face
        REQUIRE(m.face_attrs[0].mirror_edges[1].has_value());
        REQUIRE(m.face_attrs[0].mirror_edges[1].value().fid(m) == 1);
        REQUIRE(m.face_attrs[0].mirror_edges[1].value().vid(m) == 3);
        // checking for first face mirroring face
        REQUIRE(m.face_attrs[1].mirror_edges[1].has_value());
        REQUIRE(m.face_attrs[1].mirror_edges[1].value().fid(m) == 0);
        REQUIRE(m.face_attrs[1].mirror_edges[1].value().vid(m) == 0);
        // checking for second face
        REQUIRE(m.face_attrs[2].mirror_edges[1].has_value());
        REQUIRE(m.face_attrs[2].mirror_edges[1].value().fid(m) == 3);
        REQUIRE(m.face_attrs[2].mirror_edges[1].value().vid(m) == 1);
        // checking for second face mirroring face
        REQUIRE(m.face_attrs[3].mirror_edges[1].has_value());
        REQUIRE(m.face_attrs[3].mirror_edges[1].value().fid(m) == 2);
        REQUIRE(m.face_attrs[3].mirror_edges[1].value().vid(m) == 1);
    }
}

TEST_CASE("test mirror edge setup")
{
    AdaptiveTessellation m;
    Eigen::MatrixXd UV;
    Eigen::MatrixXi F;
    std::filesystem::path input_mesh_path = "/home/yunfan/hemisphere.obj";
    m.create_paired_seam_mesh_with_offset(input_mesh_path.string(), UV, F);
    Eigen::MatrixXd V3d;
    Eigen::MatrixXi F3d;
    igl::read_triangle_mesh(input_mesh_path.string(), V3d, F3d);
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
    std::filesystem::path input_mesh_path = WMT_DATA_DIR "/hemisphere.obj";
    m.create_paired_seam_mesh_with_offset(input_mesh_path.string(), UV, F);
    Eigen::MatrixXd V3d;
    Eigen::MatrixXi F3d;
    igl::read_triangle_mesh(input_mesh_path.string(), V3d, F3d);
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

// TODO test set fixed for vertex that has more than 2 curveid
// all vertices that have coloring that have more than 2 vertices should be fixed
TEST_CASE("test curve fixed get_all_mirror_vids")
{
    AdaptiveTessellation m;
    Eigen::MatrixXd UV;
    Eigen::MatrixXi F;
    std::filesystem::path input_mesh_path = WMT_DATA_DIR "/hemisphere.obj";
    m.create_paired_seam_mesh_with_offset(input_mesh_path.string(), UV, F);
    m.set_fixed();
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

// TODO test uv-index to color mapping. test color to uv-index
TEST_CASE("uv-index and coloring test")
{
    AdaptiveTessellation m;
    Eigen::MatrixXd UV;
    Eigen::MatrixXi F;
    std::filesystem::path input_mesh_path = WMT_DATA_DIR "/hemisphere.obj";
    m.create_paired_seam_mesh_with_offset(input_mesh_path.string(), UV, F);
    Eigen::MatrixXd V3d;
    Eigen::MatrixXi F3d;
    igl::read_triangle_mesh(input_mesh_path.string(), V3d, F3d);
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

// TODO special case for link condition in seamed mesh. a tube with a seam edge in the middle

TEST_CASE("edge curve-id assignment")
{
    AdaptiveTessellation m;
    Eigen::MatrixXd UV;
    Eigen::MatrixXi F;
    std::filesystem::path input_mesh_path = WMT_DATA_DIR "/hemisphere.obj";
    m.create_paired_seam_mesh_with_offset(input_mesh_path.string(), UV, F);
    m.set_fixed();
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
    Eigen::MatrixXd UV;
    Eigen::MatrixXi F;
    m.create_paired_seam_mesh_with_offset("/home/yunfan/seamPyramid.obj", UV, F);

    Image image;
    image.load(
        "/home/yunfan/seamPyramid_height_10.exr",
        WrappingMode::MIRROR_REPEAT,
        WrappingMode::MIRROR_REPEAT);

    m.mesh_parameters.m_position_normal_paths = {"/home/yunfan/seamPyramid_position.exr",
                                                 "/home/yunfan/seamPyramid_normal_smooth.exr"};
    assert(m.check_mesh_connectivity_validity());
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
    m.consolidate_mesh();
    m.write_displaced_obj("split_result.obj", m.mesh_parameters.m_displacement);
    m.write_obj("split_result_2d.obj");
    m.smooth_all_vertices();
    m.write_displaced_obj("smooth_result.obj", m.mesh_parameters.m_displacement);
    m.write_obj("smooth_result_2d.obj");
}