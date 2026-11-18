#include <wmtk/utils/AMIPS.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <wmtk/components/prismatic_mesh/bad_quality_relocation.hpp>
#include <wmtk/components/prismatic_mesh/junction_protection.hpp>
#include <wmtk/components/prismatic_mesh/prism_jacobian.hpp>
#include <wmtk/components/prismatic_mesh/relocation_projection.hpp>

using namespace wmtk::components::prismatic_mesh;
using wmtk::MatrixXd;
using wmtk::Vector3d;
namespace {
using Tet = std::array<size_t, 4>;
double q(const PrismaticMeshInput& m, size_t tid)
{
    std::array<double, 12> p;
    for (int j = 0; j < 4; ++j)
        for (int k = 0; k < 3; ++k) p[3 * j + k] = m.vertices(m.tetrahedra(tid, j), k);
    return wmtk::AMIPS_energy(p);
}
PrismaticMeshInput
mesh(MatrixXd p, std::vector<Tet> tets, const std::vector<int64_t>& parents, double h)
{
    PrismaticMeshInput m;
    m.vertices = std::move(p);
    const size_t n = m.vertices.rows();
    m.corr_input_vertex = parents;
    m.corr_input_vid.assign(n, -1);
    m.vertex_tags.assign(n, 1);
    m.vertex_component_ids.assign(n, -1);
    m.singular_vertex_tags.assign(n, -1);
    m.optimal_normals = MatrixXd::Zero(n, 3);
    m.target_positions = m.vertices;
    m.input_to_offset_vertices.resize(n);
    m.target_thickness = h;
    for (size_t v = 0; v < n; ++v) m.source_vertex_ids.push_back(100 + v);
    for (size_t v = 0; v < n; ++v) {
        if (parents[v] < 0) {
            m.input_vertices.push_back(v);
            continue;
        }
        m.vertex_tags[v] = 2;
        m.offset_vertices.push_back(v);
        m.corr_input_vid[v] = m.source_vertex_ids[parents[v]];
        m.vertex_component_ids[v] = parents[v];
        m.singular_vertex_tags[v] = 0;
        m.optimal_normals.row(v) << 0, 0, 1;
        m.target_positions.row(v) = m.vertices.row(parents[v]) + h * Vector3d::UnitZ().transpose();
        m.input_to_offset_vertices[parents[v]].push_back(v);
    }
    m.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i) {
        if (!tet_volume_above_threshold(m.vertices, tets[i], 0)) std::swap(tets[i][0], tets[i][1]);
        REQUIRE(tet_volume_above_threshold(m.vertices, tets[i], 0));
        for (int j = 0; j < 4; ++j) m.tetrahedra(i, j) = tets[i][j];
    }
    m.mesh = std::make_unique<wmtk::TetMesh>();
    m.mesh->init_with_isolated_vertices(n, tets);
    m.input_cells.assign(tets.size(), 0);
    m.offset_tet_tags.assign(tets.size(), 1);
    m.junction_protection = std::make_shared<JunctionProtection>();
    return m;
}
PrismDominantMesh all_tets(const PrismaticMeshInput& m)
{
    PrismDominantMesh h;
    for (size_t i = 0; i < static_cast<size_t>(m.tetrahedra.rows()); ++i) {
        std::vector<size_t> v;
        for (int j = 0; j < 4; ++j) v.push_back(m.tetrahedra(i, j));
        h.cells.push_back({HybridCellType::Tetrahedron, v, {i}});
        h.source_tet_to_cell.push_back(i);
    }
    return h;
}
PrismaticMeshInput edge_fixture(bool obstacle = false)
{
    MatrixXd p(obstacle ? 13 : 10, 3);
    p.topRows(10) << 0, 0, 0, 1, 0, 0, 0, 1e-6, 1, 1e-6, 0, 1, 0, 1, 1, 10, 0, 0, 11, 0, 0, 10, 1,
        0, 10, 0, 1, 0, -1, 1;
    std::vector<Tet> t{{0, 2, 3, 4}, {0, 1, 4, 9}, {5, 6, 7, 8}};
    std::vector<int64_t> parent{-1, -1, 0, 0, 1, -1, -1, -1, 5, -1};
    if (obstacle) {
        p.bottomRows(3) << -.01, 0, 0, -.01, 1, 0, -.01, 0, 2;
        t.push_back({10, 11, 12, 2});
        parent.insert(parent.end(), 3, -1);
    }
    auto m = mesh(p, t, parent, 1);
    m.offset_tet_tags[1] = -1;
    if (obstacle) m.offset_tet_tags.back() = -1;
    return m;
}
PrismaticMeshInput domain_fixture(bool obstacle = false)
{
    MatrixXd p(obstacle ? 10 : 7, 3);
    p.topRows(7) << 0, 0, 0, -1, -1, 1, 1, -1, 1, 1, 1, 1, -1, 1, 1, -1e-6, 0, 1, 1e-6, 0, 1;
    std::vector<Tet>
        t{{0, 5, 1, 2}, {0, 5, 2, 6}, {0, 6, 2, 3}, {0, 6, 3, 4}, {0, 6, 4, 5}, {0, 5, 4, 1}};
    std::vector<int64_t> parents{-1, 0, 0, 0, 0, 0, 0};
    if (obstacle) {
        p.bottomRows(3) << -.01, -.6, .8, -.01, .6, .8, -.01, 0, 1.8;
        t.push_back({7, 8, 9, 5});
        parents.insert(parents.end(), 3, -1);
    }
    auto m = mesh(p, t, parents, 1);
    m.fixed_background_vertices.assign(m.vertices.rows(), false);
    for (size_t v : {1, 2, 3, 4}) m.fixed_background_vertices[v] = true;
    if (obstacle) m.offset_tet_tags.back() = -1;
    return m;
}
PrismaticMeshInput reconstructed_fixture()
{
    MatrixXd p(6, 3);
    p << 13.5206386627, 1.2, 7.9597942562, 13.5206386627, 2., 7.788358002970294, 13.5206386627,
        1.4299932790601808, 7.551711364106658, 13.532849893533708, 1.2122633050757745,
        7.886547642499853, 13.595901995262373, 2., 7.788358002970294, 13.595901995262373,
        1.4299932790601808, 7.551711364106658;
    auto m = mesh(
        p,
        {{0, 2, 1, 4}, {0, 2, 4, 5}, {0, 3, 5, 4}},
        {-1, -1, -1, 0, 1, 2},
        .0752633325623725);
    for (size_t v : {3, 4, 5}) {
        const Vector3d d = m.vertices.row(v) - m.vertices.row(v - 3);
        m.optimal_normals.row(v) = d.normalized();
        m.target_positions.row(v) =
            m.vertices.row(v - 3) + m.target_thickness * d.normalized().transpose();
    }
    return m;
}
PrismDominantMesh reconstructed_hybrid()
{
    PrismDominantMesh h;
    h.cells = {
        {HybridCellType::Pyramid, {2, 5, 4, 1, 0}, {0, 1}, 7},
        {HybridCellType::Tetrahedron, {0, 3, 5, 4}, {2}, 7}};
    h.source_tet_to_cell = {0, 0, 1};
    h.report["transition_and_tet_regions"] = {
        {{"candidate", 7}, {"minimum_global_prism_jacobian", .00014433309813593033}}};
    return h;
}

} // namespace

TEST_CASE(
    "Relocation projection preserves feasible tangent motion at a displacement-ball pole",
    "[bad_quality_relocation]")
{
    using internal::project_relocation_displacement;
    const Vector3d normal = Vector3d::UnitZ();
    const Vector3d query(1, 0, 2);
    const auto projected = project_relocation_displacement(query, normal, -1, 1, 1);
    CHECK((projected - query.normalized()).norm() < 1e-14);
    CHECK(projected[0] > 0);
    CHECK((projected - query).norm() < (Vector3d(0, 0, 1) - query).norm());
    const auto clipped = project_relocation_displacement(Vector3d(2, 0, 2), normal, -.1, .2, 1);
    CHECK(std::abs(clipped[2] - .2) < 1e-14);
    CHECK(std::abs(clipped[0] - std::sqrt(.96)) < 1e-14);
    const Vector3d feasible(.1, .2, .05);
    CHECK(project_relocation_displacement(feasible, normal, -.1, .2, 1) == feasible);
}

TEST_CASE(
    "Gradient fallback escapes a nearly coplanar tet whose Newton step rounds away",
    "[bad_quality_relocation]")
{
    MatrixXd p(4, 3);
    p << -46.6890634710977, 38.21062644359702, 1.090011238474887, -46.70415646712659,
        38.24981245854277, 1.130181497824565, -46.70130513742007, 38.23626116999215,
        1.130414306276172, -46.69869136599403, 38.223895178835804, 1.1305561304077136;
    auto m = mesh(p, {{0, 1, 2, 3}}, {-1, 0, 0, 0}, .3343768960666879);
    for (size_t v : {1, 2, 3}) m.singular_vertex_tags[v] = 1;
    auto h = all_tets(m);
    const auto cells = m.tetrahedra;
    const auto correspondence = m.corr_input_vid;
    REQUIRE(q(m, 0) > 1e8);
    BadQualityRelocationOptions o;
    o.tet_face_iterations = 1;
    relocate_bad_quality_regions(m, h, 0, o);
    const auto& stage = h.report["bad_quality_relocation"]["tet_face_relocation"];
    CHECK(stage["accepted_gradient_moves"].get<size_t>() > 0);
    CHECK(stage["energy_after"].get<double>() < stage["energy_before"].get<double>() * 1e-4);
    CHECK(q(m, 0) < 100);
    CHECK(m.vertices.row(0) == p.row(0));
    CHECK(m.tetrahedra == cells);
    CHECK(m.corr_input_vid == correspondence);
    for (size_t v : {1, 2, 3}) {
        const Vector3d normal = (Vector3d(p.row(v)) - Vector3d(p.row(0))).normalized();
        const double r0 = (p.row(v) - p.row(0)).dot(normal) / m.target_thickness - 1;
        const double r1 =
            (m.vertices.row(v) - m.vertices.row(0)).dot(normal) / m.target_thickness - 1;
        CHECK(std::abs(r1) <= std::max(.1, std::abs(r0)) + 1e-10);
        CHECK((m.vertices.row(v) - p.row(v)).norm() <= .5 * m.target_thickness * (1 + 1e-10));
    }
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "Bad quality classes distinguish residual edges from valid-prism-derived tets",
    "[bad_quality_relocation]")
{
    auto a = edge_fixture();
    auto ha = all_tets(a);
    const auto ra = classify_bad_quality_regions(a, ha);
    CHECK(ra.collapse_edge_tets == std::vector<size_t>{0});
    REQUIRE(ra.collapse_edges.size() == 1);
    CHECK(ra.collapse_edges[0] == std::array<size_t, 2>{2, 3});
    CHECK(ra.prism_tets.empty());
    auto b = reconstructed_fixture();
    auto hb = reconstructed_hybrid();
    const auto rb = classify_bad_quality_regions(b, hb);
    CHECK(rb.collapse_edges.empty());
    CHECK(rb.prism_tets == std::vector<size_t>{2});
    hb.report["transition_and_tet_regions"][0]["minimum_global_prism_jacobian"] = -1.;
    const auto invalid = classify_bad_quality_regions(b, hb);
    CHECK(invalid.prism_tets.empty());
    CHECK(invalid.other_tets == std::vector<size_t>{2});
}

TEST_CASE(
    "Offset-domain AMIPS relocation moves only the two ends on the current surface",
    "[bad_quality_relocation]")
{
    auto m = domain_fixture();
    auto h = all_tets(m);
    const auto p = m.vertices;
    const auto cells = m.tetrahedra;
    double before = 0;
    for (size_t tid = 0; tid < 6; ++tid) before += q(m, tid);
    REQUIRE(q(m, 1) > 1000);
    BadQualityRelocationOptions o;
    o.collapse_edge_iterations = 30;
    relocate_bad_quality_regions(m, h, 0, o);
    double after = 0;
    for (size_t tid = 0; tid < 6; ++tid) after += q(m, tid);
    CHECK(after < before * .1);
    CHECK(q(m, 1) < 100);
    CHECK(q(m, 4) < 100);
    CHECK(m.vertices.topRows(5) == p.topRows(5));
    CHECK(m.vertices(5, 2) == 1);
    CHECK(m.vertices(6, 2) == 1);
    CHECK(m.tetrahedra == cells);
    const auto& result = h.report["bad_quality_relocation"]["collapse_edge_domain"];
    CHECK(result["accepted_moves"].get<size_t>() > 0);
    CHECK(result["energy_after"].get<double>() < result["energy_before"].get<double>());
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "Offset-domain relocation lowers total AMIPS while preserving background positivity",
    "[bad_quality_relocation]")
{
    auto m = domain_fixture(true);
    auto h = all_tets(m);
    std::vector<double> before;
    for (int tid = 0; tid < m.tetrahedra.rows(); ++tid) before.push_back(q(m, tid));
    BadQualityRelocationOptions o;
    o.collapse_edge_iterations = 30;
    relocate_bad_quality_regions(m, h, 0, o);
    CHECK(
        h.report["bad_quality_relocation"]["collapse_edge_domain"]["accepted_moves"].get<size_t>() >
        0);
    double old_energy = 0, new_energy = 0;
    for (size_t tid = 0; tid < before.size(); ++tid) {
        old_energy += before[tid];
        new_energy += q(m, tid);
    }
    CHECK(new_energy < old_energy);
    CHECK(m.vertices(5, 0) > -.01);
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "Offset-domain relocation permits individual quality increases when total AMIPS falls",
    "[bad_quality_relocation]")
{
    auto m = domain_fixture();
    auto h = all_tets(m);
    const auto original = m.vertices;
    std::vector<double> before;
    for (int tid = 0; tid < m.tetrahedra.rows(); ++tid) before.push_back(q(m, tid));
    BadQualityRelocationOptions o;
    o.amips_threshold = 3;
    o.collapse_edge_iterations = 30;
    relocate_bad_quality_regions(m, h, 0, o);
    bool increased = false;
    double old_energy = 0, new_energy = 0;
    for (size_t tid = 0; tid < before.size(); ++tid) {
        const double after = q(m, tid);
        increased = increased || after > before[tid] * (1 + 1e-6);
        old_energy += before[tid];
        new_energy += after;
    }
    CHECK(increased);
    CHECK(new_energy < old_energy * .1);
    CHECK(m.vertices.topRows(5) == original.topRows(5));
    const auto& result = h.report["bad_quality_relocation"]["collapse_edge_domain"];
    CHECK_FALSE(result["per_cell_quality_limits"].get<bool>());
    double previous = result["energy_before"].get<double>();
    for (const auto& step : result["history"]) {
        const double energy = step["energy"].get<double>();
        CHECK(energy <= previous);
        if (step["accepted"].get<size_t>() > 0) CHECK(energy < previous);
        previous = energy;
    }
    // Independently evaluated AMIPS can differ by roundoff across platforms.
    CHECK_THAT(new_energy / 3, Catch::Matchers::WithinRel(previous, 1e-12));
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "AMIPS and thickness relocation repairs the actual SIGGRAPH reconstruction example",
    "[bad_quality_relocation]")
{
    auto m = reconstructed_fixture();
    auto h = reconstructed_hybrid();
    const auto p = m.vertices;
    const auto cells = m.tetrahedra;
    const auto ids = m.corr_input_vid;
    REQUIRE(q(m, 2) > 300);
    BadQualityRelocationOptions o;
    o.prism_tet_iterations = 60;
    relocate_bad_quality_regions(m, h, 0, o);
    CHECK(q(m, 2) < 100);
    CHECK(m.vertices.topRows(3) == p.topRows(3));
    CHECK(m.tetrahedra == cells);
    CHECK(m.corr_input_vid == ids);
    for (size_t v : {3, 4, 5}) {
        const Vector3d d = m.vertices.row(v) - m.vertices.row(v - 3);
        CHECK(std::abs(d.dot(m.optimal_normals.row(v)) / m.target_thickness - 1) <= .1 + 1e-10);
        CHECK((m.vertices.row(v) - p.row(v)).norm() <= .5 * m.target_thickness * (1 + 1e-10));
    }
    const auto& report = h.report["bad_quality_relocation"]["prism_tet_relocation"];
    CHECK(report["energy_after"].get<double>() < report["energy_before"].get<double>());
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "Tet-face relocation optimizes all eligible surface vertices even below the bad threshold",
    "[bad_quality_relocation]")
{
    auto m = domain_fixture();
    m.vertices(5, 0) = -.2;
    m.vertices(6, 0) = .2;
    auto h = all_tets(m);
    const auto original = m.vertices;
    const auto cells = m.tetrahedra;
    const auto correspondence = m.corr_input_vid;
    for (size_t tid = 0; tid < 6; ++tid) REQUIRE(q(m, tid) < 100);
    BadQualityRelocationOptions o;
    o.tet_face_allow_mixed_cells = false;
    o.tet_face_iterations = 20;
    relocate_bad_quality_regions(m, h, 0, o);
    const auto& stage = h.report["bad_quality_relocation"]["tet_face_relocation"];
    CHECK(stage["selected_source_vids"] == std::vector<int64_t>{105, 106});
    CHECK(stage["accepted_moves"].get<size_t>() > 0);
    CHECK_FALSE(stage["per_cell_quality_limits"].get<bool>());
    CHECK(stage["tet_only_vertex_stars"].get<bool>());
    CHECK(m.vertices.topRows(5) == original.topRows(5));
    CHECK(m.tetrahedra == cells);
    CHECK(m.corr_input_vid == correspondence);
    double previous = stage["energy_before"].get<double>();
    for (const auto& row : stage["history"]) {
        const double next = row["energy"].get<double>();
        CHECK(next <= previous);
        previous = next;
    }
    CHECK(previous < stage["energy_before"].get<double>());
    for (size_t v : {5, 6}) {
        CHECK(
            (m.vertices.row(v) - original.row(v)).norm() <= .5 * m.target_thickness * (1 + 1e-10));
        CHECK(std::abs(m.vertices(v, 2) - 1) <= .1 + 1e-10);
    }
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "Tet-face relocation fixes vertices incident to native prisms",
    "[bad_quality_relocation]")
{
    const auto sample = reconstructed_fixture();
    MatrixXd p(9, 3);
    p.topRows(6) = sample.vertices;
    p.row(6) = p.row(3) + Vector3d(1, 0, 0).transpose();
    p.row(7) = p.row(3) + Vector3d(0, 1, 0).transpose();
    p.row(8) = p.row(3) + Vector3d(-.2, -.2, -.1).transpose();
    auto m = mesh(
        p,
        {{0, 2, 1, 4}, {0, 2, 4, 5}, {0, 3, 5, 4}, {8, 3, 6, 7}},
        {-1, -1, -1, 0, 1, 2, 8, 8, -1},
        sample.target_thickness);
    auto h = all_tets(m);
    h.cells = {
        {HybridCellType::Prism, {0, 2, 1, 3, 5, 4}, {0, 1, 2}},
        {HybridCellType::Tetrahedron, h.cells[3].vertices, {3}}};
    h.source_tet_to_cell = {0, 0, 0, 1};
    validate_prism_dominant_mesh(m, h, 0);
    const auto original = m.vertices;
    BadQualityRelocationOptions o;
    o.tet_face_allow_mixed_cells = false;
    o.tet_face_iterations = 10;
    relocate_bad_quality_regions(m, h, 0, o);
    const auto& stage = h.report["bad_quality_relocation"]["tet_face_relocation"];
    CHECK(stage["selected_source_vids"] == std::vector<int64_t>{106, 107});
    CHECK(stage["excluded_non_tet_vertices"].get<size_t>() == 1);
    CHECK(m.vertices.topRows(6) == original.topRows(6));
    CHECK(stage["accepted_moves"].get<size_t>() > 0);
    CHECK(stage["energy_after"].get<double>() < stage["energy_before"].get<double>());
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "Tet-face relocation also fixes vertices incident to native pyramids",
    "[bad_quality_relocation]")
{
    auto m = reconstructed_fixture();
    auto h = reconstructed_hybrid();
    const auto original = m.vertices;
    BadQualityRelocationOptions o;
    o.tet_face_allow_mixed_cells = false;
    o.tet_face_iterations = 10;
    relocate_bad_quality_regions(m, h, 0, o);
    const auto& stage = h.report["bad_quality_relocation"]["tet_face_relocation"];
    CHECK(stage["selected_source_vids"] == std::vector<int64_t>{103});
    CHECK(stage["excluded_non_tet_vertices"].get<size_t>() == 2);
    for (size_t v : {0, 1, 2, 4, 5}) CHECK(m.vertices.row(v) == original.row(v));
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "Mixed-star tet-face relocation moves shared corners while keeping native cells positive",
    "[bad_quality_relocation]")
{
    auto m = reconstructed_fixture();
    auto h = reconstructed_hybrid();
    std::vector<int64_t> expected{103, 104, 105};
    std::vector<size_t> shared{4, 5};
    SECTION("Pyramid corners") {}
    SECTION("Prism corner")
    {
        MatrixXd p(9, 3);
        p.topRows(6) = m.vertices;
        p.row(6) = p.row(3) + Vector3d(1, 0, 0).transpose();
        p.row(7) = p.row(3) + Vector3d(0, 1, 0).transpose();
        p.row(8) = p.row(3) + Vector3d(-.2, -.2, -.1).transpose();
        m = mesh(
            p,
            {{0, 2, 1, 4}, {0, 2, 4, 5}, {0, 3, 5, 4}, {8, 3, 6, 7}},
            {-1, -1, -1, 0, 1, 2, 8, 8, -1},
            m.target_thickness);
        h = all_tets(m);
        h.cells = {
            {HybridCellType::Prism, {0, 2, 1, 3, 5, 4}, {0, 1, 2}},
            {HybridCellType::Tetrahedron, h.cells[3].vertices, {3}}};
        h.source_tet_to_cell = {0, 0, 0, 1};
        expected = {103, 106, 107};
        shared = {3};
    }
    const auto original = m.vertices;
    const auto cells = m.tetrahedra;
    BadQualityRelocationOptions o;
    o.tet_face_iterations = 10;
    REQUIRE(o.tet_face_allow_mixed_cells);
    relocate_bad_quality_regions(m, h, 0, o);
    const auto& stage = h.report["bad_quality_relocation"]["tet_face_relocation"];
    CHECK(stage["selected_source_vids"] == expected);
    CHECK(stage["allows_mixed_native_cells"].get<bool>());
    CHECK_FALSE(stage["tet_only_vertex_stars"].get<bool>());
    CHECK(stage["excluded_non_tet_vertices"].get<size_t>() == 0);
    CHECK(stage["selected_mixed_native_vertices"].get<size_t>() == shared.size());
    CHECK(stage["energy_after"].get<double>() < stage["energy_before"].get<double>());
    bool moved_shared = false;
    for (size_t v : shared) moved_shared |= m.vertices.row(v) != original.row(v);
    CHECK(moved_shared);
    CHECK(m.tetrahedra == cells);
    for (size_t v : m.input_vertices) CHECK(m.vertices.row(v) == original.row(v));
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "Relocation is opt-in and validates its iteration and thickness options",
    "[bad_quality_relocation]")
{
    auto m = edge_fixture();
    auto h = all_tets(m);
    const auto p = m.vertices;
    BadQualityRelocationOptions o;
    relocate_bad_quality_regions(m, h, 0, o);
    CHECK(m.vertices == p);
    CHECK_FALSE(h.report.contains("bad_quality_relocation"));
    o.collapse_edge_iterations = -1;
    CHECK_THROWS(relocate_bad_quality_regions(m, h, 0, o));
    o.collapse_edge_iterations = 0;
    o.prism_tet_iterations = 201;
    CHECK_THROWS(relocate_bad_quality_regions(m, h, 0, o));
    o.prism_tet_iterations = 0;
    o.tet_face_iterations = 201;
    CHECK_THROWS(relocate_bad_quality_regions(m, h, 0, o));
    o.tet_face_iterations = 0;
    o.thickness_tolerance = 1;
    CHECK_THROWS(relocate_bad_quality_regions(m, h, 0, o));
    o.thickness_tolerance = .1;
    o.thickness_weight = 0;
    CHECK_THROWS(relocate_bad_quality_regions(m, h, 0, o));
}

TEST_CASE(
    "No-collapse junction endpoints can relocate in their shared offset domain",
    "[bad_quality_relocation]")
{
    auto m = domain_fixture();
    for (size_t v : {5, 6}) {
        m.junction_protection->offset_vertices.insert(v);
        m.junction_protection->offset_positions.emplace(v, m.vertices.row(v));
        REQUIRE_FALSE(junction_collapse_allowed(m, v));
    }
    auto h = all_tets(m);
    h.report["junction_protection"] = junction_protection_report(m);
    const auto original = m.vertices;
    const auto cells = m.tetrahedra;
    const auto correspondence = m.corr_input_vid;
    BadQualityRelocationOptions o;
    o.collapse_edge_iterations = 20;
    relocate_bad_quality_regions(m, h, 0, o);
    const auto& repair = h.report["bad_quality_relocation"];
    CHECK(repair["collapse_edge_domain"]["accepted_moves"].get<size_t>() > 0);
    CHECK(
        repair["collapse_edge_domain"]["energy_after"].get<double>() <
        repair["collapse_edge_domain"]["energy_before"].get<double>());
    CHECK(q(m, 1) < 100);
    CHECK(m.vertices.row(5) != original.row(5));
    CHECK(m.vertices.row(6) != original.row(6));
    CHECK(m.vertices.topRows(5) == original.topRows(5));
    CHECK(m.tetrahedra == cells);
    CHECK(m.corr_input_vid == correspondence);
    for (size_t v : {5, 6}) CHECK_FALSE(junction_collapse_allowed(m, v));
    CHECK(repair["no_collapse_offset_vertices_relocated"].get<size_t>() == 2);
    CHECK_FALSE(h.report["junction_protection"]["offset_positions_fixed"].get<bool>());
    CHECK(h.report["junction_protection"]["no_collapse_offsets_may_relocate"].get<bool>());
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "No-collapse junction offsets can optimize AMIPS and thickness next to a pyramid",
    "[bad_quality_relocation]")
{
    auto m = reconstructed_fixture();
    for (size_t v : {3, 4, 5}) {
        m.junction_protection->offset_vertices.insert(v);
        m.junction_protection->offset_positions.emplace(v, m.vertices.row(v));
    }
    auto h = reconstructed_hybrid();
    h.report["junction_protection"] = junction_protection_report(m);
    const auto original = m.vertices;
    const auto cells = m.tetrahedra;
    const auto correspondence = m.corr_input_vid;
    BadQualityRelocationOptions o;
    o.tet_face_iterations = 20;
    relocate_bad_quality_regions(m, h, 0, o);
    const auto& repair = h.report["bad_quality_relocation"];
    const auto& stage = repair["tet_face_relocation"];
    CHECK(stage["selected_source_vids"] == std::vector<int64_t>{103, 104, 105});
    CHECK(stage["energy_after"].get<double>() < stage["energy_before"].get<double>());
    CHECK(stage["accepted_moves"].get<size_t>() > 0);
    CHECK(repair["no_collapse_offset_vertices_relocated"].get<size_t>() > 0);
    CHECK_FALSE(h.report["junction_protection"]["offset_positions_fixed"].get<bool>());
    CHECK(m.vertices.topRows(3) == original.topRows(3));
    CHECK(m.tetrahedra == cells);
    CHECK(m.corr_input_vid == correspondence);
    for (size_t v : {3, 4, 5}) {
        CHECK_FALSE(junction_collapse_allowed(m, v));
        const double r0 = (Vector3d(original.row(v)) - Vector3d(original.row(v - 3)))
                                  .dot(m.optimal_normals.row(v)) /
                              m.target_thickness -
                          1;
        const double r1 = (Vector3d(m.vertices.row(v)) - Vector3d(m.vertices.row(v - 3)))
                                  .dot(m.optimal_normals.row(v)) /
                              m.target_thickness -
                          1;
        CHECK(std::abs(r1) <= std::max(.1, std::abs(r0)) + 1e-10);
        CHECK(
            (m.vertices.row(v) - original.row(v)).norm() <= .5 * m.target_thickness * (1 + 1e-10));
    }
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "Offset-domain repair does not depend on target positions or normals",
    "[bad_quality_relocation]")
{
    auto a = domain_fixture();
    auto b = domain_fixture();
    b.target_thickness = 0;
    b.target_positions.setConstant(1000);
    b.optimal_normals.setZero();
    for (size_t v : {5, 6}) b.singular_vertex_tags[v] = 1;
    auto ha = all_tets(a), hb = all_tets(b);
    BadQualityRelocationOptions o;
    o.collapse_edge_iterations = 20;
    relocate_bad_quality_regions(a, ha, 0, o);
    relocate_bad_quality_regions(b, hb, 0, o);
    CHECK(a.vertices == b.vertices);
    CHECK(
        hb.report["bad_quality_relocation"]["collapse_edge_domain"]["uses_target_positions"] ==
        false);
    CHECK(
        hb.report["bad_quality_relocation"]["collapse_edge_domain"]["uses_thickness_energy"] ==
        false);
}

TEST_CASE(
    "Bad-tet transition growth freezes the seeds before visiting one face ring",
    "[bad_quality_relocation]")
{
    MatrixXd p(7, 3);
    p << 0, 0, 0, -1, 0, 1, 1, 0, 1, 0, 1e-6, 1, -1, 1, 1, -2, 1, 1, -1, 2, 1;
    auto m = mesh(p, {{0, 1, 2, 3}, {0, 1, 3, 4}, {0, 4, 5, 6}}, {-1, 0, 0, 0, 0, 0, 0}, 1);
    auto h = all_tets(m);
    const auto labels = m.vertex_tags;
    const auto corr = m.corr_input_vid;
    REQUIRE(q(m, 0) > 100);
    REQUIRE(q(m, 1) < 100);
    REQUIRE(q(m, 2) < 100);
    BadQualityRelocationOptions options;
    expand_bad_tet_transition_regions(m, h, 0, options, 1);
    const auto& retry = h.report["bad_tet_transition_expansion"];
    REQUIRE(retry["performed_rounds"] == 1);
    const auto& step = retry["rounds"][0];
    CHECK(step["seed_offset_source_vids"] == std::vector<int64_t>{101, 102, 103});
    CHECK(step["one_ring_offset_source_vids"] == std::vector<int64_t>{101, 102, 103, 104});
    CHECK(step["one_ring_faces_source_vids"].size() == 2);
    CHECK(m.vertices == p);
    CHECK(m.vertex_tags == labels);
    CHECK(m.corr_input_vid == corr);
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "Transition expansion skips clean shell tets and validates its budget",
    "[bad_quality_relocation]")
{
    auto m = domain_fixture();
    m.vertices(5, 0) = -.2;
    m.vertices(6, 0) = .2;
    auto h = all_tets(m);
    const auto p = m.vertices;
    BadQualityRelocationOptions options;
    expand_bad_tet_transition_regions(m, h, 0, options, 0);
    CHECK_FALSE(h.report.contains("bad_tet_transition_expansion"));
    expand_bad_tet_transition_regions(m, h, 0, options, 1);
    CHECK(h.report["bad_tet_transition_expansion"]["performed_rounds"] == 0);
    CHECK(h.report["bad_tet_transition_expansion"]["stop_reason"] == "no_bad_actual_shell_tets");
    CHECK(m.vertices == p);
    CHECK_THROWS(expand_bad_tet_transition_regions(m, h, 0, options, -1));
    CHECK_THROWS(expand_bad_tet_transition_regions(m, h, 0, options, 6));
}

TEST_CASE(
    "Transition expansion options are honored by the component memory route",
    "[bad_quality_relocation]")
{
    auto m = domain_fixture();
    wmtk::MeshMap raw;
    write_prismatic_mesh(m, "mesh.vtu", nullptr, true, &raw);
    nlohmann::json config = {
        {"application", "prismatic_mesh"},
        {"input", "construction"},
        {"output", "mesh.vtu"},
        {"iterations", 0},
        {"prism_dominant", true},
        {"collapse_edge_relocation_iterations", 0},
        {"tet_face_relocation_iterations", 0},
        {"bad_tet_transition_expansion_rounds", 1}};
    const auto result = prismatic_mesh(config, {{"construction", raw.at("mesh.vtu")}});
    CHECK(result.second["hybrid"]["bad_tet_transition_expansion"]["performed_rounds"] == 1);
    config["prism_dominant"] = false;
    CHECK_THROWS(prismatic_mesh(config, {{"construction", raw.at("mesh.vtu")}}));
}

TEST_CASE(
    "Prism-tet relocation honors a zero normal-thickness tolerance",
    "[bad_quality_relocation]")
{
    auto m = reconstructed_fixture();
    auto h = reconstructed_hybrid();
    const double initial = q(m, 2);
    BadQualityRelocationOptions o;
    o.prism_tet_iterations = 60;
    o.thickness_tolerance = 0;
    relocate_bad_quality_regions(m, h, 0, o);
    CHECK(q(m, 2) < initial);
    for (size_t v : {3, 4, 5}) {
        const Vector3d d = m.vertices.row(v) - m.vertices.row(v - 3);
        CHECK(std::abs(d.dot(m.optimal_normals.row(v)) / m.target_thickness - 1) < 1e-9);
    }
    validate_prism_dominant_mesh(m, h, 0);
}

TEST_CASE(
    "Relocation JSON options and entry-region labels survive the memory route",
    "[bad_quality_relocation]")
{
    auto m = domain_fixture();
    wmtk::MeshMap raw;
    write_prismatic_mesh(m, "mesh.vtu", nullptr, true, &raw);
    const auto result = prismatic_mesh(
        {{"application", "prismatic_mesh"},
         {"input", "construction"},
         {"output", "mesh.vtu"},
         {"iterations", 0},
         {"thicknessratio", 1.},
         {"prism_dominant", true},
         {"tet_face_relocation_iterations", 0},
         {"collapse_edge_relocation_iterations", 1}},
        {{"construction", raw.at("mesh.vtu")}});
    const auto& repair = result.second["hybrid"]["bad_quality_relocation"];
    CHECK(repair["collapse_edge_before"]["count"].get<size_t>() > 0);
    CHECK(repair["collapse_edge_domain"]["accepted_moves"].get<size_t>() > 0);
    const auto& data = result.first.at("mesh");
    const auto& fields = data.cells.at(wmtk::CellType::Tetrahedron).attributes;
    REQUIRE(fields.count("bad_quality_region") == 1);
    CHECK(fields.at("bad_quality_region").doubles().maxCoeff() == 1);
    CHECK_THROWS(prismatic_mesh(
        {{"application", "prismatic_mesh"},
         {"input", "construction"},
         {"output", "mesh.vtu"},
         {"iterations", 0},
         {"prism_dominant", false},
         {"prism_tet_relocation_iterations", 1}},
        {{"construction", raw.at("mesh.vtu")}}));
    CHECK_THROWS(prismatic_mesh(
        {{"application", "prismatic_mesh"},
         {"input", "construction"},
         {"output", "mesh.vtu"},
         {"iterations", 0},
         {"prism_dominant", false},
         {"tet_face_relocation_iterations", 1}},
        {{"construction", raw.at("mesh.vtu")}}));
}

TEST_CASE(
    "Hybrid component defaults match the accepted explicit twenty plus twenty pipeline",
    "[bad_quality_relocation]")
{
    auto m = reconstructed_fixture();
    wmtk::MeshMap raw;
    write_prismatic_mesh(m, "mesh.vtu", nullptr, true, &raw);
    nlohmann::json config = {
        {"application", "prismatic_mesh"},
        {"input", "construction"},
        {"output", "mesh.vtu"},
        {"iterations", 0},
        {"prism_dominant", true}};
    const auto defaults = prismatic_mesh(config, {{"construction", raw.at("mesh.vtu")}});
    const auto& repair = defaults.second["hybrid"]["bad_quality_relocation"];
    CHECK(repair["collapse_edge_domain"]["requested_iterations"] == 20);
    CHECK(repair["prism_tet_relocation"]["requested_iterations"] == 0);
    CHECK(repair["tet_face_relocation"]["requested_iterations"] == 20);
    CHECK(repair["tet_face_relocation"]["allows_mixed_native_cells"].get<bool>());
    config["collapse_edge_relocation_iterations"] = 20;
    config["prism_tet_relocation_iterations"] = 0;
    config["tet_face_relocation_iterations"] = 20;
    config["tet_face_relocation_allow_mixed_cells"] = true;
    const auto explicit_settings = prismatic_mesh(config, {{"construction", raw.at("mesh.vtu")}});
    CHECK(repair == explicit_settings.second["hybrid"]["bad_quality_relocation"]);
    CHECK(defaults.first.at("mesh").V == explicit_settings.first.at("mesh").V);
    config["collapse_edge_relocation_iterations"] = 0;
    config["tet_face_relocation_iterations"] = 0;
    const auto disabled = prismatic_mesh(config, {{"construction", raw.at("mesh.vtu")}});
    CHECK_FALSE(disabled.second["hybrid"].contains("bad_quality_relocation"));
    config["prism_dominant"] = false;
    config.erase("collapse_edge_relocation_iterations");
    config.erase("tet_face_relocation_iterations");
    const auto pure_tets = prismatic_mesh(config, {{"construction", raw.at("mesh.vtu")}});
    CHECK_FALSE(pure_tets.second.contains("hybrid"));
    CHECK(pure_tets.first.at("mesh").V == m.vertices);
}

TEST_CASE(
    "Default thickness postprocessing reports a skip when target thickness is unavailable",
    "[bad_quality_relocation]")
{
    auto m = domain_fixture();
    wmtk::MeshMap raw;
    write_prismatic_mesh(m, "mesh.vtu", nullptr, true, &raw);
    nlohmann::json config = {
        {"application", "prismatic_mesh"},
        {"input", "construction"},
        {"output", "mesh.vtu"},
        {"iterations", 0},
        {"prism_dominant", true}};
    const auto result = prismatic_mesh(config, {{"construction", raw.at("mesh.vtu")}});
    const auto& stage = result.second["hybrid"]["bad_quality_relocation"]["tet_face_relocation"];
    CHECK(stage["requested_iterations"] == 20);
    CHECK(stage["actual_iterations"] == 0);
    CHECK(stage["skipped_reason"] == "no_positive_target_thickness");
    config["tet_face_relocation_iterations"] = 20;
    CHECK_THROWS(prismatic_mesh(config, {{"construction", raw.at("mesh.vtu")}}));
}

TEST_CASE(
    "Edges without an offset triangle domain are reported without a spring fallback",
    "[bad_quality_relocation]")
{
    MatrixXd p(4, 3);
    p << 0, 0, 0, 0, 1, 0, 0, 0, 1, 1e-6, 0, 1;
    auto m = mesh(p, {{0, 1, 2, 3}}, {-1, -1, 0, 0}, 1);
    auto h = all_tets(m);
    BadQualityRelocationOptions o;
    o.collapse_edge_iterations = 20;
    relocate_bad_quality_regions(m, h, 0, o);
    CHECK(m.vertices == p);
    const auto& rejected =
        h.report["bad_quality_relocation"]["collapse_edge_domain"]["rejected_domains"];
    REQUIRE(rejected.size() == 1);
    CHECK(rejected[0]["reason"] == "empty_or_invalid_patch");
}
