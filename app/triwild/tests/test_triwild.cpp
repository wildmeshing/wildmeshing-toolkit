#include <TriWild.h>
#include <igl/facet_components.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <remeshing/UniformRemeshing.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <wmtk/utils/BoundaryParametrization.h>
#include <wmtk/utils/DisplacementBicubic.h>
#include <wmtk/utils/Image.h>
#include <wmtk/utils/MipMap.h>
#include <wmtk/utils/autodiff.h>
#include <wmtk/utils/bicubic_interpolation.h>
#include <catch2/catch.hpp>
#include <finitediff.hpp>
#include <functional>
#include <wmtk/utils/ManifoldUtils.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>

using namespace wmtk;
using namespace triwild;

template <class T>
using RowMatrix2 = Eigen::Matrix<T, Eigen::Dynamic, 2, Eigen::RowMajor>;
using Index = uint64_t;
using Scalar = double;

TEST_CASE("triwild_collapse", "[triwild_collapse][.]")
{
    // dummy case. Collapse 5 times. 1 tri
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/test_triwild_collapse_onboundary.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }

    // without envelop. boundary is locked, nothing changes
    // center vertex have 7 tris
    triwild::TriWild m;
    m.mesh_parameters.m_target_l = 1.;
    m.create_mesh(V, F);
    m.mesh_parameters.m_bnd_freeze = true;
    m.set_energy(std::make_unique<wmtk::AMIPS>());

    for (auto& t : m.get_faces()) {
        assert(m.get_quality(t) > 0);
    }
    m.collapse_all_edges();
    m.consolidate_mesh();
    for (auto v : m.get_vertices()) {
        if (v.vid(m) == 2) REQUIRE(m.get_valence_for_vertex(v) == 7);
    }
    m.write_obj("triwild_collapse_freeze.obj");
}

TEST_CASE("triwild_split", "[triwild_split][.]")
{
    // dummy case. swap 5 times. 1 tri
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/test_triwild.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    std::vector<Eigen::Vector3d> v(V.rows());
    std::vector<std::array<size_t, 3>> tri(F.rows());
    for (int i = 0; i < V.rows(); i++) {
        v[i] = V.row(i);
    }
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) tri[i][j] = (size_t)F(i, j);
    }

    // edges are split regardless of envelope or not
    triwild::TriWild m;
    m.mesh_parameters.m_target_l = 1.;
    m.create_mesh(V, F);
    m.set_energy(std::make_unique<wmtk::AMIPS>());

    m.split_all_edges();
    REQUIRE(m.vert_capacity() == 12);
    for (auto f : m.get_faces()) {
        REQUIRE(!m.is_inverted(f));
    }
    m.write_obj("triwild_split.obj");
}

TEST_CASE("triwild_swap", "[triwild_swap][.]")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/test_triwild_swap.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);

    // without envelop. boundary is locked, nothing changes
    // center vertex have 7 tris
    TriWild m;
    m.mesh_parameters.m_target_l = 5e-2;
    m.create_mesh(V, F);
    m.set_energy(std::make_unique<wmtk::AMIPS>());

    for (auto& t : m.get_faces()) {
        REQUIRE(m.get_quality(t) > 0);
    }
    m.swap_all_edges();
    for (auto v : m.get_vertices()) {
        if (v.vid(m) == 2) REQUIRE(m.get_valence_for_vertex(v) == 7);
    }
    m.write_obj("triwild_swap_freeze.obj");
    // with envelop. can be swapped
    // center vertex have 6 tris after swap
    TriWild m2;
    m2.mesh_parameters.m_target_l = 5e-2;
    m2.create_mesh(V, F);
    m2.mesh_parameters.m_bnd_freeze = true;
    m2.set_energy(std::make_unique<wmtk::AMIPS>());

    for (auto& t : m2.get_faces()) {
        REQUIRE(m2.get_quality(t) > 0);
    }
    m2.swap_all_edges();
    for (auto v : m2.get_vertices()) {
        if (v.vid(m2) == 2) REQUIRE(m2.get_valence_for_vertex(v) == 6);
    }
    m.write_obj("triwild_swap_envelop.obj");
}

TEST_CASE("triwild_improve")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/test_triwild.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(path, V, F);
    REQUIRE(ok);
    TriWild m;
    m.mesh_parameters.m_target_l = 0.5;
    m.mesh_parameters.m_stop_energy = 2.0;
    m.create_mesh(V, F);
    m.mesh_parameters.m_bnd_freeze = true;
    m.set_energy(std::make_unique<wmtk::AMIPS>());

    m.mesh_improvement(10);
    m.write_obj("triwild_improve_freezebnd.obj");
}

TEST_CASE("AABB")
{
    const std::string root(WMT_DATA_DIR);
    const std::string path = root + "/test_triwild.obj";
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    bool ok = igl::read_triangle_mesh(path, V, F);

    REQUIRE(ok);
    TriWild m;
    m.create_mesh(V, F);
    m.set_projection();

    auto result = m.mesh_parameters.m_get_closest_point(Eigen::RowVector2d(-0.7, 0.6));
    REQUIRE(result == Eigen::RowVector2d(-1, 0.6));
}

TEST_CASE("edge_length_energy_smooth_constant")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    DiffScalarBase::setVariableCount(2);
    auto displacement = []([[maybe_unused]] const DScalar& u,
                           [[maybe_unused]] const DScalar& v) -> DScalar { return DScalar(1); };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u), DScalar(v)).getValue());
        return p;
    };
    TriWild m;
    m.create_mesh(V, F);
    // set the 3 feature point as not fixed
    for (auto v : m.get_vertices()) {
        m.vertex_attrs[v.vid(m)].fixed = false;
    }
    m.set_parameters(2, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, false);

    m.smooth_all_vertices();
    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_constant_noboundary.obj",
        displacement_double);
}

TEST_CASE("edge_length_energy_smooth_linear")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    auto displacement = [](const DScalar& u, [[maybe_unused]] const DScalar& v) -> DScalar {
        return DScalar(u);
    };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u), DScalar(v)).getValue());
        return p;
    };
    TriWild m;
    m.create_mesh(V, F);
    // set the 3 feature point as not fixed
    for (auto v : m.get_vertices()) {
        m.vertex_attrs[v.vid(m)].fixed = false;
    }
    m.set_parameters(2, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, false);

    m.smooth_all_vertices();
    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_linear_noboundary.obj",
        displacement_double);
}

TEST_CASE("edge_length_energy_smooth_dramatic_linear")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    DiffScalarBase::setVariableCount(2);
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        return DScalar(10 * u);
    };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u), DScalar(v)).getValue());
        return p;
    };
    TriWild m;
    m.create_mesh(V, F);
    // set the 3 feature point as not fixed
    for (auto v : m.get_vertices()) {
        m.vertex_attrs[v.vid(m)].fixed = false;
    }
    m.set_parameters(2, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, false);
    // set the 3 feature point as not fixed
    for (auto v : m.get_vertices()) {
        m.vertex_attrs[v.vid(m)].fixed = false;
    }
    m.smooth_all_vertices();
    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_dramatic_linear_noboundary.obj",
        displacement_double);
}

TEST_CASE("edge_length_energy_constant_remesh")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    DiffScalarBase::setVariableCount(2);
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar { return DScalar(1); };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u), DScalar(v)).getValue());
        return p;
    };
    TriWild m;
    m.create_mesh(V, F);
    m.set_parameters(0.5, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, true);
    m.mesh_improvement(3);
    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_constant_remesh_yesboundary.obj",
        displacement_double);
}
TEST_CASE("edge_length_energy_one_triangle_linear_remesh")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar { return DScalar(u); };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u), DScalar(v)).getValue());
        return p;
    };
    TriWild m;
    m.create_mesh(V, F);
    m.set_parameters(0.5, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, true);

    m.mesh_improvement(3);

    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_linear_remesh_yesboundary.obj",
        displacement_double);
}
TEST_CASE("edge_length_energy_one_triangle_dramatic_linear_remesh")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    // Eigen::MatrixXd V;
    // Eigen::MatrixXi F;
    // bool ok = igl::read_triangle_mesh("after_split_0.obj", V, F);
    // assert(ok);
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        return DScalar(10 * u);
    };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u), DScalar(v)).getValue());
        return p;
    };
    TriWild m;
    // create the json file to record logs
    std::ofstream js_o("dramatic_linear_nobnd.json");
    m.create_mesh(V, F);
    m.set_parameters(1, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, false);
    for (auto v : m.get_vertices()) {
        m.vertex_attrs[v.vid(m)].fixed = false;
    }
    m.mesh_improvement(100);

    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_dramatic_linear_remesh_noboundary.obj",
        displacement_double);
    js_o << std::setw(4) << m.mesh_parameters.js_log << std::endl;
    js_o.close();
}
TEST_CASE("edge_length_energy_one_triangle_smooth_remesh")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        return sin(u * M_PI) + cos(v * M_PI);
    };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u) / 10., DScalar(v) / 10.).getValue());
        return p;
    };
    TriWild m;
    // create the json file to record logs
    std::ofstream js_o("smooth_yesbnd.json");
    m.create_mesh(V, F);
    m.set_parameters(0.1, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, true);
    for (auto v : m.get_vertices()) {
        REQUIRE(m.vertex_attrs[v.vid(m)].fixed);
    }
    m.mesh_improvement(3);

    m.write_displaced_obj(
        "twoandahalf_edge_length_one_triangle_smooth_remesh_yesoboundary.obj",
        displacement_double);
    js_o << std::setw(4) << m.mesh_parameters.js_log << std::endl;
    js_o.close();
}

TEST_CASE("smoothing_gradient_debug")
{
    using DScalar = wmtk::TwoAndAHalf::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    // Eigen::MatrixXd V;
    // Eigen::MatrixXi F;
    // bool ok = igl::read_triangle_mesh("after_split_0.obj", V, F);
    // assert(ok);
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        return DScalar(10 * u);
    };
    auto displacement_double = [&displacement](double u, double v) -> double {
        return displacement(DScalar(u), DScalar(v)).getValue();
    };
    auto displacement_vector = [&displacement](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement(DScalar(u), DScalar(v)).getValue());
        return p;
    };
    TriWild m;
    // create the json file to record logs
    std::ofstream js_o("gradient_debug_yesbnd.json");
    m.create_mesh(V, F);
    m.set_parameters(1, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, true);
    for (auto v : m.get_vertices()) {
        // m.vertex_attrs[v.vid(m)].fixed = false;
    }
    m.gradient_debug(1000);

    m.write_displaced_obj("smooth_gradient_debug_yesbnd.obj", displacement_double);
    js_o << std::setw(4) << m.mesh_parameters.js_log << std::endl;
    js_o.close();
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
    bnd.construct_boudaries(V, F);

    REQUIRE(bnd.m_boundaries.size() == 1);
    REQUIRE(bnd.m_arclengths.size() == 1);

    REQUIRE(bnd.m_arclengths[0].size() == 4);

    REQUIRE(bnd.m_arclengths[0][0] == 0);
    REQUIRE(bnd.m_arclengths[0][1] == 10);
    REQUIRE(bnd.m_arclengths[0][2] == 10 + 10 * sqrt(2));
    REQUIRE(bnd.m_arclengths[0][3] == 10 + 10 + 10 * sqrt(2));

    double t;

    Eigen::Vector2d test_v0(0, 0);
    auto t0 = bnd.uv_to_t(test_v0);
    REQUIRE(t0 == 0.);
    auto v0 = bnd.t_to_uv(0, t0);
    REQUIRE(v0 == test_v0);
    auto ij0 = bnd.uv_to_ij(v0, t);
    assert(t == t0);
    REQUIRE(ij0.first == 0);
    REQUIRE(ij0.second == 0);

    Eigen::Vector2d test_v1(5, 0);
    auto t1 = bnd.uv_to_t(test_v1);
    REQUIRE(t1 == 5.);
    auto v1 = bnd.t_to_uv(0, t1);
    REQUIRE(v1 == test_v1);
    auto ij1 = bnd.uv_to_ij(v1, t);
    REQUIRE(t == t1);
    REQUIRE(ij1.first == 0);
    REQUIRE(ij1.second == 0);

    Eigen::Vector2d test_v2(5, 5);
    auto t2 = bnd.uv_to_t(test_v2);
    REQUIRE(t2 == 10 + 5. * sqrt(2));
    auto v2 = bnd.t_to_uv(0, t2);
    REQUIRE(v2 == test_v2);
    auto ij2 = bnd.uv_to_ij(v2, t);
    REQUIRE(t == t2);
    REQUIRE(ij2.first == 0);
    REQUIRE(ij2.second == 1);

    Eigen::Vector2d test_v3(0, 0.1);
    auto t3 = bnd.uv_to_t(test_v3);
    REQUIRE(t3 == 10 + 10. * sqrt(2) + 9.9);
    auto v3 = bnd.t_to_uv(0, t3);
    REQUIRE((v3 - test_v3).stableNorm() < 1e-8);
    auto ij3 = bnd.uv_to_ij(v3, t);
    REQUIRE(t == t3);
    REQUIRE(ij3.first == 0);
    REQUIRE(ij3.second == 2);
}

TEST_CASE("boundary parameter smooth")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    TriWild m;
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
    m.set_parameters(4, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, true);

    for (auto v : m.get_vertices()) {
        REQUIRE(m.vertex_attrs[v.vid(m)].t >= 0);
    }
    REQUIRE(m.mesh_parameters.m_boundary.m_arclengths.size() != 0);
    REQUIRE(m.mesh_parameters.m_boundary.m_boundaries.size() != 0);
    m.smooth_all_vertices();
    m.write_displaced_obj("smooth_new_boundary.obj", displacement_double);

    for (auto v : m.get_vertices()) {
        auto v_project = m.mesh_parameters.m_get_closest_point(m.vertex_attrs[v.vid(m)].pos);
        REQUIRE((v_project.transpose() - m.vertex_attrs[v.vid(m)].pos).squaredNorm() < 1e-5);
    }
}

TEST_CASE("boundary parameter split")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;

    TriWild m;
    m.create_mesh(V, F);
    m.set_projection();
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar { return DScalar(1); };
    auto displacement_double = [](double u, double v) -> double { return 1; };
    auto displacement_vector = [&displacement_double](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, displacement_double(u, v));
        return p;
    };

    m.set_parameters(4, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, true);

    for (auto v : m.get_vertices()) {
        REQUIRE(m.vertex_attrs[v.vid(m)].t >= 0);
    }
    REQUIRE(m.mesh_parameters.m_boundary.m_arclengths.size() != 0);
    REQUIRE(m.mesh_parameters.m_boundary.m_boundaries.size() != 0);
    for (auto f : m.get_faces()) REQUIRE(!m.is_inverted(f));
    m.split_all_edges();
    m.write_displaced_obj("split_new_boundary.obj", displacement_double);

    for (auto e : m.get_edges()) {
        if (m.is_boundary_edge(e)) {
            auto v1_pos = m.mesh_parameters.m_boundary.t_to_uv(0, m.vertex_attrs[e.vid(m)].t);
            auto v2_pos = m.mesh_parameters.m_boundary.t_to_uv(
                0,
                m.vertex_attrs[e.switch_vertex(m).vid(m)].t);
            REQUIRE(
                (m.mesh_parameters.m_get_closest_point(v1_pos) - v1_pos.transpose()).squaredNorm() <
                1e-5);
            REQUIRE(
                (m.mesh_parameters.m_get_closest_point(v2_pos) - v2_pos.transpose()).squaredNorm() <
                1e-5);
        }
    }
}

TEST_CASE("boundary parameter collapse")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh("../build_release/after_split_7.obj", V, F);
    assert(ok);

    TriWild m;
    m.create_mesh(V, F);
    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        return DScalar(10 * u);
    };
    auto displacement_double = [](double u, double v) -> double { return 10 * u; };
    auto displacement_vector = [](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, 10 * u);
        return p;
    };

    m.set_parameters(1, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, true);

    for (auto v : m.get_vertices()) {
        REQUIRE(m.vertex_attrs[v.vid(m)].t >= 0);
    }
    REQUIRE(m.mesh_parameters.m_boundary.m_arclengths.size() != 0);
    REQUIRE(m.mesh_parameters.m_boundary.m_boundaries.size() != 0);
    m.collapse_all_edges();
    m.consolidate_mesh();
    m.write_displaced_obj("collapse_new_boundary.obj", displacement_double);

    for (auto e : m.get_edges()) {
        if (m.is_boundary_edge(e)) {
            auto v1_pos = m.mesh_parameters.m_boundary.t_to_uv(0, m.vertex_attrs[e.vid(m)].t);
            auto v2_pos = m.mesh_parameters.m_boundary.t_to_uv(
                0,
                m.vertex_attrs[e.switch_vertex(m).vid(m)].t);
            REQUIRE(
                (m.mesh_parameters.m_get_closest_point(v1_pos) - v1_pos.transpose()).squaredNorm() <
                1e-5);
            REQUIRE(
                (m.mesh_parameters.m_get_closest_point(v2_pos) - v2_pos.transpose()).squaredNorm() <
                1e-5);
        }
    }
}

TEST_CASE("energy gradient")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;
    DiffScalarBase::setVariableCount(2);

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh(
        "twoandahalf_edge_length_one_triangle_dramatic_linear_remesh_yesboundary.obj",
        V,
        F);
    assert(ok);

    TriWild m;
    m.create_mesh(V, F);

    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        return DScalar(10 * u);
    };
    auto displacement_vector = [](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, 10 * u);
        return p;
    };
    m.set_parameters(1, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, true);

    Eigen::VectorXd v_flat;
    m.flatten_dofs(v_flat);
    Eigen::VectorXd finitediff_grad = Eigen::VectorXd::Zero(m.get_vertices().size() * 2);

    std::function<double(const Eigen::VectorXd&)> f =
        [&m](const Eigen::VectorXd& v_flat) -> double { return m.get_mesh_energy(v_flat); };
    fd::finite_gradient(v_flat, f, finitediff_grad, fd::SECOND, 1e-2);

    for (auto v : m.get_vertices()) {
        auto autodiff = m.get_one_ring_energy(v);
        wmtk::logger().info(
            "boundary {} autodiff {} finitediff {}, {}",
            m.is_boundary_vertex(v),
            autodiff.second,
            finitediff_grad[v.vid(m) * 2],
            finitediff_grad[v.vid(m) * 2 + 1]);
    }
}

TEST_CASE("gradient")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    bool ok = igl::read_triangle_mesh("split_new_boundary.obj", V, F);
    assert(ok);
    TriWild m;
    m.create_mesh(V, F);

    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar {
        return DScalar(10 * u);
    };
    auto displacement_vector = [](double u, double v) -> Eigen::Vector3d {
        Eigen::Vector3d p(u, v, 10 * u);
        return p;
    };
    m.set_parameters(1, displacement, EDGE_LEN_TYPE::LINEAR3D, ENERGY_TYPE::EDGE_LENGTH, true);

    Eigen::VectorXd v_flat, finitediff_grad;
    m.flatten_dofs(v_flat);
    finitediff_grad.resize(v_flat.size());
    std::function<double(const Eigen::VectorXd&)> f =
        [&m](const Eigen::VectorXd& v_flat) -> double {
        double energy = 0;
        for (int i = 0; i < v_flat.size(); i++) {
            if (v_flat(i) == std::numeric_limits<double>::infinity()) continue;
            wmtk::logger().info("v_flat({}) {}", i, v_flat(i));
            energy += v_flat(i);
        }
        return energy;
    };
    fd::finite_gradient(v_flat, f, finitediff_grad, fd::SECOND, 1e-2);
}

TEST_CASE("line_parametrization")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    DiffScalarBase::setVariableCount(2);

    DScalar t(0, 0);
    Eigen::Vector2d A(10, 0);
    Eigen::Vector2d B(7.5, 2.5);
    Eigen::Matrix<DScalar, 2, 1> ad_n;
    ad_n << t * (B - A).normalized()(0), t * (B - A).normalized()(1);
    wmtk::logger().info("ad_n {}", ad_n);
    wmtk::logger().info(
        "but they are actually {} {}",
        t * (B - A).normalized()(0),
        t * (B - A).normalized()(1));
    auto x = A(0) + t * (B - A).normalized()(0);
    auto y = A(1) + t * (B - A).normalized()(1);
    wmtk::logger().info("tmpA_x {} ", x);
    wmtk::logger().info("tmpA_y {}", y);
    auto f = pow(5 - x, 2) + pow(0 - y, 2);
    f = pow(f - 1, 2);
    wmtk::logger().info(f.getValue());
    wmtk::logger().info(f.getGradient());
    wmtk::logger().info(f);

    Eigen::VectorXd fd_t, finitediff_grad;
    finitediff_grad.resize(1);
    fd_t.resize(1);
    fd_t(0) = 0.;
    std::function<double(const Eigen::VectorXd&)> fd_f =
        [&A, &B](const Eigen::VectorXd& fd_t) -> double {
        double energy = 0;
        auto n = (B - A).normalized();
        wmtk::logger()
            .info("fd_t {} A {} B {}", fd_t(0), (A(0) + fd_t(0) * n(0)), (A(1) + fd_t(0) * n(1)));
        energy += pow(5 - (A(0) + fd_t(0) * n(0)), 2) + pow(0 - (A(1) + fd_t(0) * n(1)), 2);
        energy = pow(energy - 1, 2);
        wmtk::logger().info("energy {}", energy);
        return energy;
    };
    fd::finite_gradient(fd_t, fd_f, finitediff_grad, fd::SECOND, 1e-2);
}

// TODO: Try out sin(x) with periodic boundary cond + autodiff + gradient

TEST_CASE("remeshing using image data")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    int w, h;
    w = 512;
    h = 512;
    Image image(h, w);
    auto displacement_double = [](const double& u, const double& v) -> float {
        return static_cast<float>(10 * sin(M_PI * u) * cos(M_PI * v));
        // return 10 * u;
    };
    auto displacement_dscalar = [](const DScalar& u, const DScalar& v) -> DScalar {
        return 10 * sin(M_PI * u) * cos(M_PI * v);
        // return 10 * u;
    };
    image.set(displacement_double);
    image.save("tryout.exr");
    Image image2(512, 512);
    image2.load("tryout.exr", WrappingMode::MIRROR_REPEAT, WrappingMode::MIRROR_REPEAT);

    auto displacement = [&image2](const DScalar& u, const DScalar& v) -> DScalar {
        return image2.get(u, v);
    };
    auto displacement_image_double = [&image2](const double& u, const double& v) -> double {
        return image2.get(u / 10., v / 10.);
    };
    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    TriWild m;
    m.create_mesh(V, F);
    m.set_parameters(
        0.01,
        image2,
        WrappingMode::MIRROR_REPEAT,
        EDGE_LEN_TYPE::LINEAR3D,
        ENERGY_TYPE::EDGE_LENGTH,
        true);
    m.mesh_improvement(3);

    m.write_displaced_obj("remesh_from_image_linear.obj", displacement_image_double);
}

TEST_CASE("fixed corner")
{
    Eigen::MatrixXd V(3, 2);
    V.row(0) << 0, 0;
    V.row(1) << 10, 0;
    V.row(2) << 0, 10;
    Eigen::MatrixXi F(1, 3);
    F.row(0) << 0, 1, 2;
    TriWild m;
    m.create_mesh(V, F);
    for (auto v : m.get_vertices()) {
        REQUIRE(m.vertex_attrs[v.vid(m)].fixed);
    }
}

TEST_CASE("stripe")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    Image image(1024, 1024);
    image.load(
        "/home/yunfan/data/plastic_stripes_Height.exr",
        WrappingMode::MIRROR_REPEAT,
        WrappingMode::MIRROR_REPEAT);
    auto displacement = [&image](const DScalar& u, const DScalar& v) -> DScalar {
        return image.get(u, v);
    };
    auto displacement_image_double = [&image](const double& u, const double& v) -> double {
        return (image.get(u, v));
    };

    TriWild m;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh("/home/yunfan/data/input.obj", V, F);
    m.create_mesh(V, F);
    for (auto f : m.get_faces()) {
        REQUIRE(!m.is_inverted(f));
    }
    REQUIRE(m.invariants(m.get_faces()));

    wmtk::logger().info("#v {}, #f {} ", m.vert_capacity(), m.tri_capacity());
    m.set_image_function(image, WrappingMode::MIRROR_REPEAT);
    m.set_parameters(
        0.1,
        image,
        WrappingMode::MIRROR_REPEAT,
        EDGE_LEN_TYPE::N_IMPLICIT_POINTS,
        ENERGY_TYPE::EDGE_LENGTH,
        true);
    m.mesh_improvement(3);
    m.write_displaced_obj("stripe_final.obj", m.mesh_parameters.m_project_to_3d);
}

TEST_CASE("implicit points")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    TriWild m;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh("/home/yunfan/data/moonface_output.obj", V, F);
    m.create_mesh(V, F);

    auto displacement = [](const DScalar& u, const DScalar& v) -> DScalar { return DScalar(1.); };
    m.set_parameters(
        0.05,
        displacement,
        EDGE_LEN_TYPE::N_IMPLICIT_POINTS,
        ENERGY_TYPE::EDGE_LENGTH,
        true);
    int edge_cnt = 0;
    for (auto e : m.get_edges()) {
        auto length2d = m.get_length2d(e.vid(m), e.switch_vertex(m).vid(m));
        auto length3d = m.get_length3d(e.vid(m), e.switch_vertex(m).vid(m));
        if (pow((length2d - length3d), 2) > 1e-6)
            wmtk::logger().info(
                "{} {}",
                m.vertex_attrs[e.vid(m)].pos,
                m.vertex_attrs[e.switch_vertex(m).vid(m)].pos);
        REQUIRE(pow((length2d - length3d), 2) < 1e-6);
        edge_cnt++;
    }
    wmtk::logger().info(edge_cnt);
}

TEST_CASE("exact length")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;
    Image image(512, 512);
    image.load(
        "/home/yunfan/data/plastic_stripes_Height.exr",
        WrappingMode::MIRROR_REPEAT,
        WrappingMode::MIRROR_REPEAT);
    auto displacement = [&image](const DScalar& u, const DScalar& v) -> DScalar {
        return image.get(u, v);
    };
    auto displacement_image_double = [&image](const double& u, const double& v) -> double {
        return (image.get(u, v));
    };

    TriWild m;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh("/home/yunfan/data/input.obj", V, F);
    m.create_mesh(V, F);
    m.set_image_function(image, WrappingMode::MIRROR_REPEAT);

    m.set_parameters(
        0.05,
        displacement,
        EDGE_LEN_TYPE::PT_PER_PIXEL,
        ENERGY_TYPE::EDGE_LENGTH,
        true);

    for (auto e : m.get_edges()) {
        auto length = m.get_length_1ptperpixel(e.vid(m), e.switch_vertex(m).vid(m));
        auto lengthmipmap = m.get_length_mipmap(e.vid(m), e.switch_vertex(m).vid(m));
        auto length3d = m.get_length3d(e.vid(m), e.switch_vertex(m).vid(m));
        wmtk::logger().info(
            "length_exact {} lengthMIPMAP {} length3d {} between {}, {}",
            length,
            lengthmipmap,
            length3d,
            e.vid(m),
            e.switch_vertex(m).vid(m));
    }
}

TEST_CASE("mipmap")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;

    auto displacement_double = [&](const double& u, const double& v) -> float { return 10 * u; };
    Image image(1024, 1024);
    image.set(displacement_double);
    image.save("drlin.exr");

    MipMap mipmap(image);
    REQUIRE(mipmap.level() == 11);
    for (int i = 0; i < mipmap.level(); i++) {
        auto tmp_image = mipmap.get_image(i);
        tmp_image.save(fmt::format("drlin_{:04d}.exr", i));
    }

    // now test the length of the edges using a dummy example
    TriWild m;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh("/home/yunfan/data/input.obj", V, F);
    m.create_mesh(V, F);
    for (auto f : m.get_faces()) {
        REQUIRE(!m.is_inverted(f));
    }
    REQUIRE(m.invariants(m.get_faces()));
    auto displacement = [&image](const DScalar& u, const DScalar& v) -> DScalar {
        return image.get(u, v);
    };
    m.set_parameters(
        0.1,
        image,
        WrappingMode::MIRROR_REPEAT,
        EDGE_LEN_TYPE::PT_PER_PIXEL,
        ENERGY_TYPE::EDGE_LENGTH,
        true);
    m.mesh_improvement(1);
    m.write_displaced_obj("mipmap_out.obj", m.mesh_parameters.m_project_to_3d);
}

TEST_CASE("accuracy split")
{
    Image image1(100, 100);
    auto displacement_double2 = [](const double& u, const double& v) -> double {
        return sin(M_PI * u);
    };
    image1.set(displacement_double2);
    TriWild m;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh("/home/yunfan/data/input.obj", V, F);
    m.create_mesh(V, F);
    m.set_parameters(
        0.001,
        image1,
        WrappingMode::MIRROR_REPEAT,
        EDGE_LEN_TYPE::ACCURACY,
        ENERGY_TYPE::EDGE_QUADRATURE,
        true);
    m.split_all_edges();
    m.consolidate_mesh();
    m.write_displaced_obj("accuracy_guided_split.obj", m.mesh_parameters.m_project_to_3d);

    // m.set_parameters(
    //     0.1,
    //     image1,
    //     WrappingMode::MIRROR_REPEAT,
    //     EDGE_LEN_TYPE::PT_PER_PIXEL,
    //     ENERGY_TYPE::EDGE_LENGTH,
    //     true);
    // m.split_all_edges();
    // m.consolidate_mesh();
    // m.write_displaced_obj("quality_then.obj", m.mesh_parameters.m_project_to_3d);
}

TEST_CASE("accuracy smooth")
{
    Image image1(100, 100);
    auto displacement_double2 = [](const double& u, const double& v) -> double {
        return sin(M_PI * u);
    };
    image1.set(displacement_double2);
    TriWild m;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh("accuracy_guided_split.obj", V, F);
    m.create_mesh(V, F);
    m.set_parameters(
        0.01,
        image1,
        WrappingMode::MIRROR_REPEAT,
        EDGE_LEN_TYPE::ACCURACY,
        ENERGY_TYPE::EDGE_QUADRATURE,
        true);
    m.smooth_all_vertices();
    m.write_displaced_obj("accuracy_guided_smooth.obj", m.mesh_parameters.m_project_to_3d);
}

TEST_CASE("quality split comparison")
{
    Image image1(10, 10);
    auto displacement_double2 = [](const double& u, const double& v) -> double {
        return sin(M_PI * u);
    };
    image1.set(displacement_double2);
    TriWild m;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh("/home/yunfan/data/input.obj", V, F);
    m.create_mesh(V, F);
    m.set_parameters(
        0.1,
        image1,
        WrappingMode::MIRROR_REPEAT,
        EDGE_LEN_TYPE::PT_PER_PIXEL,
        ENERGY_TYPE::EDGE_LENGTH,
        true);
    m.split_all_edges();
    m.consolidate_mesh();
    m.write_displaced_obj("quality_guided_split.obj", m.mesh_parameters.m_project_to_3d);
}

TEST_CASE("accuracy image operations")
{
    using DScalar = wmtk::EdgeLengthEnergy::DScalar;
    Image image(512, 512);
    image.load(
        "/home/yunfan/data/plastic_stripes_Height.exr",
        WrappingMode::MIRROR_REPEAT,
        WrappingMode::MIRROR_REPEAT);
    TriWild m;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::read_triangle_mesh("/home/yunfan/data/input.obj", V, F);
    m.create_mesh(V, F);
    m.set_parameters(
        0.005,
        image,
        WrappingMode::MIRROR_REPEAT,
        EDGE_LEN_TYPE::ACCURACY,
        ENERGY_TYPE::EDGE_QUADRATURE,
        true);
    m.split_all_edges();
    m.write_displaced_obj("image_accuracy_split.obj", m.mesh_parameters.m_project_to_3d);
    // m.swap_all_edges();
    // m.write_displaced_obj("image_accuracy_swap.obj", m.mesh_parameters.m_project_to_3d);
    // m.smooth_all_vertices();
    // m.write_displaced_obj("image_accuracy_smooth.obj", m.mesh_parameters.m_project_to_3d);
}