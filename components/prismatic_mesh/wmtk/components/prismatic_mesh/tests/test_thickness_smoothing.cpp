#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <wmtk/components/prismatic_mesh/junction_protection.hpp>
#include <wmtk/components/prismatic_mesh/rod_mesh.hpp>
#include <wmtk/components/prismatic_mesh/thickness_smoothing.hpp>

using namespace wmtk::components::prismatic_mesh;
using wmtk::Vector3d;

namespace {
using Tet = std::array<size_t, 4>;
PrismaticMeshInput fixture(bool rod, bool blocker = false)
{
    PrismaticMeshInput m;
    m.vertices.resize(blocker ? 7 : 6, 3);
    if (rod) {
        m.vertices.topRows(6) << 0, 0, 0, .4, 0, 0, .2, std::sqrt(3.) * .2, 0, 0, 0, 1, .4, 0, 1,
            .2, std::sqrt(3.) * .2, .2;
    } else {
        m.vertices << 0, 0, 0, 1, 0, 0, .5, std::sqrt(3.) / 2, 0, .3, .1, .35, 1.3, .1, .5, .8,
            std::sqrt(3.) / 2 + .1, .7;
    }
    m.vertex_tags = rod ? std::vector<int>{1, 2, 2, 1, 2, 2} : std::vector<int>{1, 1, 1, 2, 2, 2};
    m.corr_input_vertex =
        rod ? std::vector<int64_t>{-1, 0, 0, -1, 3, 3} : std::vector<int64_t>{-1, -1, -1, 0, 1, 2};
    std::vector<Tet> tets{{0, 1, 2, 5}, {0, 1, 5, 4}, {0, 3, 4, 5}};
    if (blocker) {
        const Vector3d a = m.vertices.row(3), b = m.vertices.row(4), c = m.vertices.row(5);
        m.vertices.row(6) =
            ((a + b + c) / 3 + 1e-6 * (b - a).cross(c - a).normalized()).transpose();
        m.vertex_tags.push_back(2);
        m.corr_input_vertex.push_back(3);
        tets.push_back({3, 4, 5, 6});
    }
    for (size_t v = 0; v < static_cast<size_t>(m.vertices.rows()); ++v) {
        m.source_vertex_ids.push_back(100 + v);
        m.corr_input_vid.push_back(m.corr_input_vertex[v] < 0 ? -1 : 100 + m.corr_input_vertex[v]);
        (m.vertex_tags[v] == 1 ? m.input_vertices : m.offset_vertices).push_back(v);
    }
    m.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i) {
        REQUIRE(tet_volume_above_threshold(m.vertices, tets[i], 0));
        for (size_t j = 0; j < 4; ++j) m.tetrahedra(i, j) = tets[i][j];
    }
    m.input_cells.assign(tets.size(), 0);
    m.offset_tet_tags.assign(tets.size(), 1);
    m.mesh = std::make_unique<wmtk::TetMesh>();
    m.mesh->init_with_isolated_vertices(m.vertices.rows(), tets);
    m.junction_protection = std::make_shared<JunctionProtection>();
    if (rod) {
        m.rod_state = std::make_shared<RodMeshState>();
        auto& s = *m.rod_state;
        s.radius = .3;
        s.input_vertices = {0, 3};
        s.input_edges = {{0, 3}};
        for (size_t center : {0, 3}) {
            RodFrame f;
            f.center = center;
            f.valid = true;
            f.ring = {center + 1, center + 2};
            s.frames.push_back(f);
            s.frame_targets[center + 1] = Vector3d(m.vertices.row(center)) + Vector3d(.3, 0, 0);
            s.frame_targets[center + 2] =
                Vector3d(m.vertices.row(center)) + Vector3d(.15, std::sqrt(3.) * .15, 0);
        }
    } else {
        m.target_thickness = .6;
        m.singular_vertex_tags.assign(6, -1);
        m.optimal_normals = wmtk::MatrixXd::Zero(6, 3);
        m.target_positions = m.vertices;
        for (size_t v : {3, 4, 5}) {
            m.singular_vertex_tags[v] = 0;
            m.optimal_normals.row(v) << 0, 0, 1;
            m.target_positions.row(v) = m.vertices.row(v - 3) + Vector3d(0, 0, .6).transpose();
        }
    }
    return m;
}
PrismDominantMesh hybrid(const PrismaticMeshInput& m)
{
    PrismDominantMesh h;
    h.cells.push_back({HybridCellType::Prism, {0, 1, 2, 3, 4, 5}, {0, 1, 2}});
    h.source_tet_to_cell = {0, 0, 0};
    if (m.tetrahedra.rows() == 4) {
        h.cells.push_back({HybridCellType::Tetrahedron, {3, 4, 5, 6}, {3}});
        h.source_tet_to_cell.push_back(1);
    }
    return h;
}
} // namespace

TEST_CASE(
    "Final shell smoothing fits normal thickness and improves the native prism",
    "[thickness_smoothing]")
{
    auto m = fixture(false);
    auto h = hybrid(m);
    const auto before = m.vertices;
    const auto tets = m.tetrahedra;
    const auto ids = m.corr_input_vid;
    const double q = prism_amips(m.vertices, h.cells[0]);
    smooth_thickness_and_prisms(m, h, 0, 60);
    CHECK(prism_amips(m.vertices, h.cells[0]) < .9 * q);
    for (int v = 3; v < 6; ++v) CHECK(std::abs(m.vertices(v, 2) - .6) < .002);
    CHECK((m.vertices.topRows(3) - before.topRows(3)).norm() == 0);
    CHECK(m.tetrahedra == tets);
    CHECK(m.corr_input_vid == ids);
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "Coupled rod fitting releases a thin neighboring tet without a tet quality objective",
    "[thickness_smoothing]")
{
    auto m = fixture(true, true);
    auto h = hybrid(m);
    const auto before = m.vertices;
    auto isolated_move = m.vertices;
    isolated_move.row(5) = m.rod_state->frame_targets.at(5).transpose();
    REQUIRE_FALSE(tet_volume_above_threshold(isolated_move, {3, 4, 5, 6}, 0));
    const double q = prism_amips(m.vertices, h.cells[0]);
    smooth_thickness_and_prisms(m, h, 0, 100);
    CHECK(std::abs(m.vertices(5, 2) - 1) < .02);
    CHECK((m.vertices.row(6) - before.row(6)).norm() > .02);
    CHECK(prism_amips(m.vertices, h.cells[0]) < q);
    CHECK((m.vertices.row(0) - before.row(0)).norm() == 0);
    CHECK((m.vertices.row(3) - before.row(3)).norm() == 0);
    CHECK(h.report["thickness_smoothing"]["tet_objective"] == "validity_only");
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "Final smoothing preserves protected entry positions and zero rounds are inert",
    "[thickness_smoothing]")
{
    auto m = fixture(false);
    auto h = hybrid(m);
    const auto before = m.vertices;
    smooth_thickness_and_prisms(m, h, 0, 0);
    CHECK(m.vertices == before);
    m.junction_protection->offset_vertices.insert(3);
    smooth_thickness_and_prisms(m, h, 0, 10);
    CHECK((m.vertices.row(3) - before.row(3)).norm() == 0);
    CHECK_THROWS_AS(smooth_thickness_and_prisms(m, h, 0, -1), std::invalid_argument);
    CHECK_THROWS_AS(smooth_thickness_and_prisms(m, h, 0, 201), std::invalid_argument);
}

TEST_CASE(
    "Prism shape smoothing preserves an already attained shell thickness",
    "[thickness_smoothing]")
{
    auto m = fixture(false);
    for (int v = 3; v < 6; ++v) m.vertices(v, 2) = m.target_thickness;
    auto h = hybrid(m);
    const double initial = prism_amips(m.vertices, h.cells[0]);
    smooth_thickness_and_prisms(m, h, 0, 60);
    CHECK(prism_amips(m.vertices, h.cells[0]) < initial);
    for (int v = 3; v < 6; ++v) CHECK(std::abs(m.vertices(v, 2) - m.target_thickness) < 1e-12);
}

TEST_CASE(
    "Native prism AMIPS has the equilateral normalization and rejects inversion",
    "[thickness_smoothing]")
{
    wmtk::MatrixXd p(6, 3);
    p << 0, 0, 0, 1, 0, 0, .5, std::sqrt(3.) / 2, 0, 0, 0, 1, 1, 0, 1, .5, std::sqrt(3.) / 2, 1;
    HybridCell c{HybridCellType::Prism, {0, 1, 2, 3, 4, 5}, {0, 1, 2}};
    CHECK(std::abs(prism_amips(p, c) - 3) < 1e-12);
    p *= 2.7;
    p.rowwise() += Vector3d(3, -2, 8).transpose();
    CHECK(std::abs(prism_amips(p, c) - 3) < 1e-12);
    p.col(0) *= -1;
    CHECK_FALSE(std::isfinite(prism_amips(p, c)));
}

TEST_CASE(
    "An existing below-floor reference tet stays unchanged in final smoothing",
    "[thickness_smoothing]")
{
    auto m = fixture(false);
    auto h = hybrid(m);
    const auto before = m.vertices;
    smooth_thickness_and_prisms(m, h, 1, 5);
    CHECK(m.vertices == before);
    CHECK(h.report["thickness_smoothing"]["fixed_below_floor_vertices"] == 6);
}
