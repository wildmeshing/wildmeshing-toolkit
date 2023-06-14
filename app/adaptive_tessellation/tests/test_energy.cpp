#include <AdaptiveTessellation.h>
#include <igl/predicates/predicates.h>
#include <igl/read_triangle_mesh.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <wmtk/utils/BoundaryParametrization.h>
#include <wmtk/utils/autodiff.h>
#include <catch2/catch.hpp>
#include <finitediff.hpp>
#include <functional>
#include <wmtk/utils/TriQualityUtils.hpp>

using namespace wmtk;
using namespace adaptive_tessellation;
template <class T>
using RowMatrix2 = Eigen::Matrix<T, Eigen::Dynamic, 2, Eigen::RowMajor>;
using Index = uint64_t;
using Scalar = double;
std::function<bool(std::array<double, 6>&)> is_inverted = [](auto& tri) {
    Eigen::Vector2d a, b, c;
    a << tri[0], tri[1];
    b << tri[2], tri[3];
    c << tri[4], tri[5];
    auto res = igl::predicates::orient2d(a, b, c);
    return (res != igl::predicates::Orientation::POSITIVE);
};
std::function<bool(std::array<double, 6>&)> is_degenerate = [](auto& tri) {
    Eigen::Vector3d a, b, c;
    a << tri[2] - tri[0], tri[3] - tri[1], 0.;
    b << tri[4] - tri[0], tri[5] - tri[1], 0.;
    c << tri[2] - tri[4], tri[3] - tri[5], 0;
    auto area = (a.cross(b)).norm();
    auto e = a.norm() + b.norm();
    e += c.norm();
    return (std::pow(area, 2) < 1e-5 || std::pow((area / e), 2) < 1e-2);
};
auto array_norm = [](const std::array<double, 6>& a, const std::array<double, 6>& b) {
    double ret;
    for (int i = 0; i < 6; i++) {
        ret += std::pow(a[i] - b[i], 2);
    }
    return std::sqrt(ret);
};

TEST_CASE("amips energy")
{
    Eigen::MatrixXd V(3, 2);
    Eigen::MatrixXi F(1, 3);
    V << -1, 1, 1, 1, -1, -1;
    F << 0, 1, 2;
    Boundary b;
    b.construct_boundaries(V, F, {}, {});

    AMIPS amips;
    DofsToPositions dof_to_pos(b, 0);
    DofVector dofx;
    dofx = Eigen::Vector2d(0, 0);
    SECTION("unit equlateral triangle")
    // using the unit equilateral traingle,
    // should have minimum energy 2
    {
        wmtk::State state = {};
        state.scaling = 1.;
        state.idx = 0;
        state.dofx = dofx;
        state.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(1, 0, 1. / 2., sqrt(3) / 2.);

        amips.eval(state, dof_to_pos);
        REQUIRE(abs(state.value - 2.0) < 1e-10);
    }
    SECTION("random equlateral triangle")
    // using a random equilateral traingle that doesn't have unit length
    // amips is invariant to triangle edge length, should return minimum energy 2
    {
        wmtk::State state = {};
        state.scaling = 1.;
        state.idx = 0;
        state.dofx = dofx;
        state.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(5, 0, 5. / 2., sqrt(3) * 5 / 2.);

        amips.eval(state, dof_to_pos);
        REQUIRE(abs(state.value - 2.0) < 1e-10);
    }
    SECTION("random triangle")
    // any random triangle should have amips energy >= 2
    {
        wmtk::State state = {};
        state.scaling = 1.;
        state.idx = 0;
        state.dofx = dofx;
        state.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(5, 2, 5. / 2., sqrt(3) * 5);

        amips.eval(state, dof_to_pos);
        REQUIRE(state.value > 2);
    }
    SECTION("rototranslation energy")
    {
        // given 2 triangles only differ by rotation,
        // AMIPS energy agianst each other shuld be minimum
        // std::array<double, 6> rand_tri = {-1, 0, 2, 0.5, 0, 8};
        // std::array<double, 6> rand_tri1 = {4., 4., 3.5, 7., -4., 5.};
        DofVector dofx = Eigen::Vector2d(-1, 0);
        wmtk::State state1 = {};
        state1.scaling = 1.;
        state1.idx = 0;
        state1.dofx = dofx;
        state1.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(2, 0.5, 0, 8);
        state1.target_triangle = {4., 4., 3.5, 7., -4., 5.};
        amips.eval(state1, dof_to_pos);

        REQUIRE(abs(state1.value - 2.0) < 1e-10);
    }
}

TEST_CASE("symdi energy")
{
    Eigen::MatrixXd V(3, 2);
    Eigen::MatrixXi F(1, 3);
    V << -1, 1, 1, 1, -1, -1;
    F << 0, 1, 2;
    Boundary b;
    b.construct_boundaries(V, F, {}, {});

    SymDi symdi;
    DofsToPositions dof_to_pos(b, 0);
    DofVector dofx;
    dofx = Eigen::Vector2d(0, 0);
    SECTION("unit equlateral triangle")
    // using the unit equilateral traingle,
    // should have minimum energy 4
    {
        wmtk::State state = {};
        state.scaling = 1.;
        state.idx = 0;
        state.dofx = dofx;
        state.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(1, 0, 1. / 2., sqrt(3) / 2.);

        symdi.eval(state, dof_to_pos);
        REQUIRE(abs(state.value - 4.0) < 1e-10);
    }
    SECTION("random equlateral triangle")
    // using a random equilateral traingle that doesn't have unit length
    // symdi is variant to triangle edge length, should return energy greater 4
    {
        wmtk::State state = {};
        state.scaling = 1.;
        state.idx = 0;
        state.dofx = dofx;
        state.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(5, 0, 5. / 2., sqrt(3) * 5 / 2.);

        symdi.eval(state, dof_to_pos);
        REQUIRE(state.value > 4.0);
    }
    SECTION("random triangle")
    // any random triangle should have symdi energy >= 4
    {
        wmtk::State state = {};
        state.scaling = 1.;
        state.idx = 0;
        state.dofx = dofx;
        state.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(5, 2, 5. / 2., sqrt(3) * 5);

        symdi.eval(state, dof_to_pos);
        REQUIRE(state.value > 4);
    }
    SECTION("rototranslation energy")
    {
        // given 2 triangles only differ by rotation,
        // AMIPS energy agianst each other shuld be minimum
        // std::array<double, 6> rand_tri = {-1, 0, 2, 0.5, 0, 8};
        // std::array<double, 6> rand_tri1 = {4., 4., 3.5, 7., -4., 5.};
        DofVector dofx = Eigen::Vector2d(-1, 0);
        wmtk::State state1 = {};
        state1.scaling = 1.;
        state1.idx = 0;
        state1.dofx = dofx;
        state1.two_opposite_vertices = Eigen::Matrix<double, 1, 4>(2, 0.5, 0, 8);
        state1.target_triangle = {4., 4., 3.5, 7., -4., 5.};
        symdi.eval(state1, dof_to_pos);

        REQUIRE(abs(state1.value - 4.0) < 1e-10);
    }
}

TEST_CASE("test_degenerate")
{
    std::array<double, 6> rand_tri = {27, 35, -14, -46, 26, 33};
    REQUIRE(is_degenerate(rand_tri));
}

TEST_CASE("2 rand tris")
{
    int pass = 0;
    Eigen::MatrixXd V(3, 2);
    Eigen::MatrixXi F(1, 3);
    V << -1, 1, 1, 1, -1, -1;
    F << 0, 1, 2;
    Boundary b;
    b.construct_boundaries(V, F, {}, {});
    DofsToPositions dof_to_pos(b, 0);

    std::array<std::shared_ptr<Energy>, 2> symdi_amips;
    std::shared_ptr<SymDi> symdi = std::make_shared<SymDi>();
    std::shared_ptr<AMIPS> amips = std::make_shared<AMIPS>();
    symdi_amips = {symdi, amips};
    std::array<double, 2> mini_energy = {4., 2.};

    for (int e = 0; e < 2; e++) {
        auto E = symdi_amips[e];
        for (int i = 0; i < 20; i++) {
            std::array<double, 6> target_tri = {0, 0, 0, 0, 0, 0};
            std::array<double, 6> input_tri = {0, 0, 0, 0, 0, 0};
            std::array<std::array<double, 6>*, 2> tris = {&target_tri, &input_tri};
            for (int i = 0; i < 2; i++) {
                while (is_degenerate(*(tris[i])) || is_inverted((*(tris[i])))) {
                    for (int j = 0; j < 6; j++) {
                        (*(tris[i]))[j] = rand() % 100 - 50;
                    }
                }
            }
            // instantiate for newton's method
            wmtk::NewtonMethodInfo nminfo;
            nminfo.curve_id = 0;
            nminfo.target_length = 1;
            nminfo.neighbors.resize(1, 4);
            wmtk::DofVector dofx;
            wmtk::State state = {};
            auto old_tri = input_tri;
            int itr = 0;
            do {
                old_tri = input_tri;
                for (int j = 0; j < 3; j++) {
                    dofx = Eigen::Vector2d(input_tri[j * 2], input_tri[j * 2 + 1]);
                    nminfo.neighbors.row(0) = Eigen::Matrix<double, 1, 4>(
                        input_tri[((j + 1) * 2) % 6],
                        input_tri[((j + 1) * 2 + 1) % 6],
                        input_tri[((j + 2) * 2) % 6],
                        input_tri[((j + 2) * 2 + 1) % 6]);

                    state.target_triangle = {target_tri[j * 2],
                                             target_tri[j * 2 + 1],
                                             target_tri[((j + 1) % 3) * 2],
                                             target_tri[((j + 1) % 3) * 2 + 1],
                                             target_tri[((j + 2) % 3) * 2],
                                             target_tri[((j + 2) % 3) * 2 + 1]};

                    wmtk::newton_method_with_fallback(*E, b, nminfo, dofx, state);
                    E->eval(state, dof_to_pos);

                    input_tri[j * 2] = dofx(0);
                    input_tri[j * 2 + 1] = dofx(1);
                }
                E->eval(state, dof_to_pos);

                std::cout << itr++ << state.gradient.stableNorm() << std::endl;
            } while (state.gradient.stableNorm() > 1e-3);
            E->eval(state, dof_to_pos);
            REQUIRE(abs(state.value - mini_energy[e]) < 1e-2);
        }
    }
}
// TODO: test for edge length error
// TODO: test for accuracy error

TEST_CASE("amips3d double")
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
        adaptive_tessellation::ENERGY_TYPE::AMIPS3D,
        adaptive_tessellation::EDGE_LEN_TYPE::LINEAR3D,
        1);
    std::vector<TriMesh::Tuple> tris_tuples = m.get_faces();
    for (int i = 0; i < tris_tuples.size(); i++) {
        wmtk::TriMesh::Tuple anchor_vertex = tris_tuples[i];
        const Eigen::Vector2d& v1 = m.get_vertex_attrs(anchor_vertex).pos;
        Eigen::Vector2d v2;
        Eigen::Vector2d v3;
        std::vector<wmtk::NewtonMethodInfo> nminfos;
        // push in current vertex's nminfo
        wmtk::NewtonMethodInfo primary_nminfo;
        primary_nminfo.neighbors.resize(1, 4);
        primary_nminfo.facet_ids.resize(1);
        primary_nminfo.facet_ids[0] = anchor_vertex.fid(m);
        std::array<wmtk::TriMesh::Tuple, 3> local_tuples = m.oriented_tri_vertices(anchor_vertex);
        for (size_t j = 0; j < 3; j++) {
            if (local_tuples[j].vid(m) == anchor_vertex.vid(m)) {
                v2 = Eigen::Vector2d(
                    m.vertex_attrs[local_tuples[(j + 1) % 3].vid(m)].pos(0),
                    m.vertex_attrs[local_tuples[(j + 1) % 3].vid(m)].pos(1));
                v3 = Eigen::Vector2d(
                    m.vertex_attrs[local_tuples[(j + 2) % 3].vid(m)].pos(0),
                    m.vertex_attrs[local_tuples[(j + 2) % 3].vid(m)].pos(1));
                primary_nminfo.neighbors.row(0) << v2(0), v2(1), v3(0), v3(1);
                auto res = igl::predicates::orient2d(v2, v3, v1);
                if (res != igl::predicates::Orientation::POSITIVE) exit(30000);

                // sanity check. Should not be inverted
            }
        }
        nminfos.emplace_back(primary_nminfo);

        wmtk::State state = {};
        state.dofx.resize(2);
        state.dofx = v1; // uv;
        state.scaling = 1.;
        state.target_triangle = std::array<double, 6>{0., 0., 1., 0., 1. / 2., sqrt(3) / 2.};

        // get current state: energy, gradient, hessiane
        wmtk::optimization_state_update(
            *m.mesh_parameters.m_energy,
            nminfos,
            m.mesh_parameters.m_boundary,
            state);
        double amips_double = m.get_amips3d_error_for_face(v1, v2, v3);
        double amips_autodiff = state.value;
        REQUIRE_THAT(amips_double, Catch::Matchers::WithinRel(amips_autodiff, 1e-10));
    }
}