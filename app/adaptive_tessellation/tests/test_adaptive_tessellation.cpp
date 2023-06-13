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
#include <wmtk/image/Image.h>
#include <wmtk/image/MipMap.h>
#include <wmtk/image/bicubic_interpolation.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <wmtk/utils/BoundaryParametrization.h>
#include <wmtk/utils/autodiff.h>
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

TEST_CASE("operations with boundary parameterization", "[.]")
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
    m.mesh_parameters.m_do_not_output = true;
    m.mesh_parameters.m_ignore_embedding = true;
    m.mesh_parameters.m_early_stopping_number = 100;
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
        m.mesh_parameters.m_boundary_parameter = false;
        for (auto v : m.get_vertices()) {
            REQUIRE(m.vertex_attrs[v.vid(m)].t >= 0);
            m.vertex_attrs[v.vid(m)].fixed = false;
        }
        REQUIRE(m.mesh_parameters.m_boundary.num_curves() != 0);
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

// NOTE: this test cannot be run becasue mesh_parameters.m_project_to_3d is invalid currently
TEST_CASE("autodiff vs finitediff", "[.]")
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
    m.mesh_parameters.m_do_not_output = true;
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

TEST_CASE("autodiff_vs_finitediff_PART_TWO", "[.]")
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
    m.mesh_parameters.m_do_not_output = true;
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar { return 10 * u; };
    m.set_parameters(4, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, true);

    for (const auto& v : m.get_vertices()) {
        REQUIRE(m.vertex_attrs[v.vid(m)].t >= 0);
        m.vertex_attrs[v.vid(m)].fixed = false;
    }
    // m.smooth_all_vertices();

    // TODO replace displacement function by the real energy function
    std::function<DScalar(const DScalar&, const DScalar&)> eval = displacement;

    std::function<double(const Eigen::VectorXd&)> fd_function =
        [&eval](const Eigen::VectorXd& p) -> double {
        const DScalar d = eval(DScalar(p[0]), DScalar(p[1]));
        return d.getValue();
    };

    {
        // evaluate gradient at v0
        Eigen::Vector2d p = V.row(0);

        Eigen::VectorXd finitediff_grad;
        fd::finite_gradient(p, fd_function, finitediff_grad, fd::SECOND, 1e-2);

        DScalar x(0, p[0]); // First variable, initialized to V(0, 0)
        DScalar y(1, p[1]); // Second variable, initialized to V(0, 1)
        const DScalar v0_displ = eval(x, y);
        const Eigen::VectorXd autodiff_grad = v0_displ.getGradient();
        spdlog::info(
            "v0 gradient FD = {}, autodiff = {}",
            finitediff_grad.transpose(),
            autodiff_grad.transpose());
    }

    // evaluate gradient for each vertex
    for (const auto& v : m.get_vertices()) {
        // TODO
    }
}

// TODO: Try out sin(x) with periodic boundary cond + autodiff + gradient
TEST_CASE("test_link_check", "[test_pre_check]")
{
    AdaptiveTessellation m;
    auto check_link_results = [&](const TriMeshTuple& edge) {
        auto vlink = TriMeshEdgeCollapseOperation::links_of_vertex(m, edge);
        auto elink = TriMeshEdgeCollapseOperation::edge_link_of_edge_vids(m, edge);

        auto svlink =
            AdaptiveTessellationPairedEdgeCollapseOperation::seamed_links_of_vertex(m, edge);
        auto selink =
            AdaptiveTessellationPairedEdgeCollapseOperation::seamed_edge_link_of_edge(m, edge);
        {
            const auto& [ee, ie] = elink;
            const auto& [see, sie] = selink;

            CHECK(ie == sie);
            CHECK(ee == see);
            spdlog::info("Edge edge links: {} seamd: {}", ee, see);
        }
        {
            CHECK(vlink.infinite_vertex == svlink.infinite_vertex);
            CHECK(vlink.vertex == svlink.vertex);
            spdlog::info("vertex links: {} seamd: {}", vlink.vertex, svlink.vertex);
            CHECK(vlink.edge == svlink.edge);
            spdlog::info("Edge links: {} seamd: {}", vlink.edge, svlink.edge);
            CHECK(vlink.infinite_edge == svlink.infinite_edge);
            spdlog::info(
                "infinite edge links: {} seamd: {}",
                vlink.infinite_edge,
                svlink.infinite_edge);
        }
    };
    SECTION("extra_face_after_collapse")
    {
        std::vector<std::array<int, 3>> tris =
            {{{1, 2, 3}}, {{0, 1, 4}}, {{0, 2, 5}}, {{0, 1, 6}}, {{0, 2, 6}}, {{1, 2, 6}}};
        Eigen::MatrixXi F(tris.size(), 3);
        for (size_t j = 0; j < tris.size(); ++j) {
            auto f = F.row(j);
            const auto& tri = tris[j];
            f = Eigen::RowVector3i::ConstMapType(tri.data());
        }
        m.create_mesh_debug(Eigen::MatrixXd(7, 3), F);
        TriMesh::Tuple edge(1, 2, 0, m);

        REQUIRE(edge.vid(m) == 1);
        REQUIRE(edge.switch_vertex(m).vid(m) == 2);

        {
            // short test for tris_bounded_by_edge
            std::vector<TriMeshTuple> faces = m.tris_bounded_by_edge(edge);
            for (const auto& f : faces) {
                REQUIRE(f.is_valid(m));
            }
            std::vector<size_t> fids;
            std::transform(
                faces.begin(),
                faces.end(),
                std::back_inserter(fids),
                [&](const TriMeshTuple& t) -> size_t { return t.fid(m); });
            REQUIRE(std::is_sorted(fids.begin(), fids.end()));
            CHECK(fids[0] == 0);
            CHECK(fids[1] == 5);
        }
        check_link_results(edge);

        REQUIRE_FALSE(
            AdaptiveTessellationPairedEdgeCollapseOperation::check_seamed_link_condition(m, edge));
    }
    SECTION("one_triangle")
    {
        std::vector<std::array<int, 3>> tris = {{{0, 1, 2}}};
        Eigen::MatrixXi F(tris.size(), 3);
        for (size_t j = 0; j < tris.size(); ++j) {
            auto f = F.row(j);
            const auto& tri = tris[j];
            f = Eigen::RowVector3i::ConstMapType(tri.data());
        }
        m.create_mesh_debug(Eigen::MatrixXd(3, 3), F);

        TriMesh::Tuple edge(0, 2, 0, m);
        assert(edge.is_valid(m));
        check_link_results(edge);
        REQUIRE_FALSE(
            AdaptiveTessellationPairedEdgeCollapseOperation::check_seamed_link_condition(m, edge));
    }
    SECTION("one_tet")
    {
        std::vector<std::array<int, 3>> tris = {{{0, 1, 2}}, {{1, 3, 2}}, {{0, 2, 3}}, {{3, 0, 1}}};
        Eigen::MatrixXi F(tris.size(), 3);
        for (size_t j = 0; j < tris.size(); ++j) {
            auto f = F.row(j);
            const auto& tri = tris[j];
            f = Eigen::RowVector3i::ConstMapType(tri.data());
        }
        m.create_mesh_debug(Eigen::MatrixXd(4, 3), F);

        TriMesh::Tuple edge(1, 0, 0, m);
        assert(edge.is_valid(m));
        check_link_results(edge);
        REQUIRE_FALSE(
            AdaptiveTessellationPairedEdgeCollapseOperation::check_seamed_link_condition(m, edge));
    }
    SECTION("non_manifold_after_collapse")
    {
        std::vector<std::array<int, 3>> tris = {{{0, 1, 5}}, {{1, 2, 5}}, {{2, 3, 5}}, {{5, 3, 4}}};
        Eigen::MatrixXi F(tris.size(), 3);
        for (size_t j = 0; j < tris.size(); ++j) {
            auto f = F.row(j);
            const auto& tri = tris[j];
            f = Eigen::RowVector3i::ConstMapType(tri.data());
        }
        m.create_mesh_debug(Eigen::MatrixXd(6, 3), F);

        TriMesh::Tuple fail_edge(5, 0, 1, m);
        check_link_results(fail_edge);
        REQUIRE_FALSE(AdaptiveTessellationPairedEdgeCollapseOperation::check_seamed_link_condition(
            m,
            fail_edge));
        TriMesh::Tuple pass_edge(0, 2, 0, m);
        check_link_results(pass_edge);
        REQUIRE(AdaptiveTessellationPairedEdgeCollapseOperation::check_seamed_link_condition(
            m,
            pass_edge));
    }
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

TEST_CASE("quickrun", "[.]")
{
    // Loading the input 2d mesh
    AdaptiveTessellation m;

    std::filesystem::path input_folder = WMTK_DATA_DIR;
    std::filesystem::path input_mesh_path = input_folder / "hemisphere_splited.obj";
    std::filesystem::path position_path = input_folder / "images/hemisphere_512_position.exr";
    std::filesystem::path normal_path =
        input_folder / "images/hemisphere_512_normal-world-space.exr";
    std::filesystem::path height_path =
        input_folder / "images/riveted_castle_iron_door_512_height.exr";

    m.mesh_preprocessing(input_mesh_path, position_path, normal_path, height_path);
    Image image;
    image.load(height_path, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);

    REQUIRE(m.check_mesh_connectivity_validity());
    m.set_parameters(
        0.00001,
        0.4,
        image,
        WrappingMode::MIRROR_REPEAT,
        SAMPLING_MODE::BICUBIC,
        DISPLACEMENT_MODE::MESH_3D,
        adaptive_tessellation::ENERGY_TYPE::QUADRICS,
        adaptive_tessellation::EDGE_LEN_TYPE::AREA_ACCURACY,
        1);
    // m.split_all_edges();
    // m.write_obj_displaced("split_result.obj");
    // m.swap_all_edges();
    // m.write_obj_displaced("swap_result.obj");
    m.smooth_all_vertices();
    m.write_obj_displaced("smooth_result_quadrics.obj");
    m.write_obj("smooth_result_2d.obj");

    AdaptiveTessellation m2;
    m2.mesh_preprocessing(input_mesh_path, position_path, normal_path, height_path);
    REQUIRE(m.check_mesh_connectivity_validity());
    // m.mesh_parameters.m_early_stopping_number = 1;

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

    m.smooth_all_vertices();
    m.write_obj_displaced("smooth_result_default.obj");
}

TEST_CASE("check curveid consistency after split")
{
    // logger().set_level(spdlog::level::trace);
    // Loading the input 2d mesh
    AdaptiveTessellation m;

    std::filesystem::path input_folder = WMTK_DATA_DIR;
    std::filesystem::path input_mesh_path = input_folder / "hemisphere_splited.obj";
    std::filesystem::path position_path = input_folder / "images/hemisphere_512_position.exr";
    std::filesystem::path normal_path =
        input_folder / "images/hemisphere_512_normal-world-space.exr";
    std::filesystem::path height_path =
        input_folder / "images/riveted_castle_iron_door_512_height.exr";


    m.mesh_preprocessing(input_mesh_path, position_path, normal_path, height_path);
    assert(m.check_mesh_connectivity_validity());
    Image image;
    image.load(height_path, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);

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

TEST_CASE("quadric split", "[.]")
{
    std::filesystem::path input_folder = WMTK_DATA_DIR;
    std::filesystem::path input_mesh_path = input_folder / "bumpyDice.obj";
    std::filesystem::path position_path = input_folder / "images/bumpyDice_128_position.exr";
    std::filesystem::path normal_path =
        input_folder / "images/bumpyDice_128_world_space_normals.exr";
    std::filesystem::path height_path = input_folder / "images/bumpyDice_128_height.exr";


    AdaptiveTessellation m;
    m.mesh_preprocessing(input_mesh_path, position_path, normal_path, height_path);
    Image image;
    image.load(height_path, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);

    m.mesh_parameters.m_output_folder = "test_quadrics_Split";
    REQUIRE(m.check_mesh_connectivity_validity());
    m.set_parameters(
        0.00001,
        0.4,
        image,
        WrappingMode::MIRROR_REPEAT,
        SAMPLING_MODE::BICUBIC,
        DISPLACEMENT_MODE::MESH_3D,
        adaptive_tessellation::ENERGY_TYPE::QUADRICS,
        adaptive_tessellation::EDGE_LEN_TYPE::TRI_QUADRICS,
        1);
    m.split_all_edges();
    m.write_obj_displaced(m.mesh_parameters.m_output_folder / "quadrics_split_result.obj");
}

TEST_CASE("old split", "[.]")
{
    std::filesystem::path input_folder = WMTK_DATA_DIR;
    std::filesystem::path input_mesh_path = input_folder / "bumpyDice.obj";
    std::filesystem::path position_path = input_folder / "images/bumpyDice_128_position.exr";
    std::filesystem::path normal_path =
        input_folder / "images/bumpyDice_128_world_space_normals.exr";
    std::filesystem::path height_path = input_folder / "images/bumpyDice_128_height.exr";

    AdaptiveTessellation m;
    m.mesh_preprocessing(input_mesh_path, position_path, normal_path, height_path);
    Image image;
    image.load(height_path, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);
    m.mesh_parameters.m_output_folder = "test_old_Split";
    REQUIRE(m.check_mesh_connectivity_validity());
    m.set_parameters(
        0.0000001,
        0.4,
        image,
        WrappingMode::MIRROR_REPEAT,
        SAMPLING_MODE::BICUBIC,
        DISPLACEMENT_MODE::MESH_3D,
        adaptive_tessellation::ENERGY_TYPE::AREA_QUADRATURE,
        adaptive_tessellation::EDGE_LEN_TYPE::AREA_ACCURACY,
        1);
    m.split_all_edges();
    m.write_obj_displaced(m.mesh_parameters.m_output_folder / "area_split_result.obj");
}

TEST_CASE("swap with accuracy passes", "[.]")
{
    std::filesystem::path input_folder = "/home/yunfan/";
    std::filesystem::path input_mesh_path = input_folder / "seamPyramid.obj";
    std::filesystem::path position_path = input_folder / "seamPyramid_position.exr";
    std::filesystem::path normal_path = input_folder / "seamPyramid_normal_smooth.exr";
    std::filesystem::path height_path = input_folder / "seamPyramid_height_10.exr";
    AdaptiveTessellation m;
    m.mesh_preprocessing(input_mesh_path, position_path, normal_path, height_path);
    Image image;
    image.load(height_path, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);
    m.mesh_parameters.m_output_folder = "test_swap_accuracy_pass";
    REQUIRE(m.check_mesh_connectivity_validity());
    m.set_parameters(
        1e-6,
        0.4,
        image,
        WrappingMode::MIRROR_REPEAT,
        SAMPLING_MODE::BICUBIC,
        DISPLACEMENT_MODE::MESH_3D,
        adaptive_tessellation::ENERGY_TYPE::AREA_QUADRATURE,
        adaptive_tessellation::EDGE_LEN_TYPE::AREA_ACCURACY,
        1);
    m.split_all_edges();
    m.swap_all_edges_accuracy_pass();
    m.write_obj_displaced(m.mesh_parameters.m_output_folder / "area_swap_accuracy_result.obj");
    // m.swap_all_edges_quality_pass();
    // m.write_obj_displaced(m.mesh_parameters.m_output_folder / "area_swap_quality_result.obj");
}

TEST_CASE("swap with quality passes", "[.]")
{
    std::filesystem::path input_folder = "/home/yunfan/";
    std::filesystem::path input_mesh_path = input_folder / "seamPyramid.obj";
    std::filesystem::path position_path = input_folder / "seamPyramid_position.exr";
    std::filesystem::path normal_path = input_folder / "seamPyramid_normal_smooth.exr";
    std::filesystem::path height_path = input_folder / "seamPyramid_height_10.exr";
    AdaptiveTessellation m;
    m.mesh_preprocessing(input_mesh_path, position_path, normal_path, height_path);
    Image image;
    image.load(height_path, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);
    m.mesh_parameters.m_output_folder = "test_swap_quality_pass";
    REQUIRE(m.check_mesh_connectivity_validity());
    m.set_parameters(
        1e-6,
        0.4,
        image,
        WrappingMode::MIRROR_REPEAT,
        SAMPLING_MODE::BICUBIC,
        DISPLACEMENT_MODE::MESH_3D,
        adaptive_tessellation::ENERGY_TYPE::AREA_QUADRATURE,
        adaptive_tessellation::EDGE_LEN_TYPE::AREA_ACCURACY,
        1);
    m.split_all_edges();
    m.swap_all_edges_quality_pass();
    m.write_obj_displaced(m.mesh_parameters.m_output_folder / "area_swap_quality_result.obj");
}

TEST_CASE("pipeline test", "[.]")
{
    std::filesystem::path input_folder = "/home/yunfan/";
    std::filesystem::path input_mesh_path =
        "../build_release_ninja/test_pipeline/swap_quality_result.obj";
    std::filesystem::path position_path = input_folder / "seamPyramid_position.exr";
    std::filesystem::path normal_path = input_folder / "seamPyramid_normal_smooth.exr";
    std::filesystem::path height_path = input_folder / "seamPyramid_height_10.exr";
    AdaptiveTessellation m;
    m.mesh_preprocessing(input_mesh_path, position_path, normal_path, height_path);
    Image image;
    image.load(height_path, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);
    m.mesh_parameters.m_output_folder = "test_pipeline";
    REQUIRE(m.check_mesh_connectivity_validity());
    m.set_parameters(
        1e-6,
        0.4,
        image,
        WrappingMode::MIRROR_REPEAT,
        SAMPLING_MODE::BICUBIC,
        DISPLACEMENT_MODE::MESH_3D,
        adaptive_tessellation::ENERGY_TYPE::AREA_QUADRATURE,
        adaptive_tessellation::EDGE_LEN_TYPE::AREA_ACCURACY,
        1);
    // m.split_all_edges();
    // m.write_obj_displaced(m.mesh_parameters.m_output_folder / "split_result.obj");
    // m.swap_all_edges_accuracy_pass();
    // m.write_obj_displaced(m.mesh_parameters.m_output_folder / "swap_accuracy_result.obj");
    // m.swap_all_edges_quality_pass();
    // m.write_obj_displaced(m.mesh_parameters.m_output_folder / "swap_quality_result.obj");
    // m.collapse_all_edges();
    // m.write_obj_displaced(m.mesh_parameters.m_output_folder / "collapse_result.obj");
    m.smooth_all_vertices();
    m.write_obj_displaced(m.mesh_parameters.m_output_folder / "smooth_result.obj");
}

TEST_CASE("early stop split test", "[.]")
{
    std::filesystem::path input_folder = "/home/yunfan/";
    std::filesystem::path input_mesh_path = input_folder / "seamPyramid.obj";
    std::filesystem::path position_path = input_folder / "seamPyramid_position.exr";
    std::filesystem::path normal_path = input_folder / "seamPyramid_normal_smooth.exr";
    std::filesystem::path height_path = input_folder / "seamPyramid_height_10.exr";
    AdaptiveTessellation m;
    m.mesh_preprocessing(input_mesh_path, position_path, normal_path, height_path);
    Image image;
    image.load(height_path, WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);
    m.mesh_parameters.m_output_folder = "test_early_stop_s1_s2";
    REQUIRE(m.check_mesh_connectivity_validity());
    m.set_parameters(
        1e-9,
        0.4,
        image,
        WrappingMode::MIRROR_REPEAT,
        SAMPLING_MODE::BICUBIC,
        DISPLACEMENT_MODE::MESH_3D,
        adaptive_tessellation::ENERGY_TYPE::AREA_QUADRATURE,
        adaptive_tessellation::EDGE_LEN_TYPE::AREA_ACCURACY,
        1);
    int cnt = 0;
    while (cnt < 10) {
        m.mesh_parameters.m_early_stopping_number = 200;
        m.split_all_edges();
        m.write_obj_displaced(
            m.mesh_parameters.m_output_folder /
            ("itr" + std::to_string(cnt) + "/split_result.obj"));
        // m.swap_all_edges_accuracy_pass();
        // m.write_obj_displaced(m.mesh_parameters.m_output_folder + "/swap_accuracy_result.obj");
        m.mesh_parameters.m_early_stopping_number = std::numeric_limits<size_t>::max();

        m.swap_all_edges_quality_pass();
        m.write_obj_displaced(
            m.mesh_parameters.m_output_folder /
            ("itr" + std::to_string(cnt) + "/swap_quality_result.obj"));
        // m.collapse_all_edges();
        // m.write_obj_displaced(m.mesh_parameters.m_output_folder / "collapse_result.obj");
        m.smooth_all_vertices();
        m.write_obj_displaced(
            m.mesh_parameters.m_output_folder /
            ("itr" + std::to_string(cnt) + "/smooth_result.obj"));
        cnt++;
    }
}
