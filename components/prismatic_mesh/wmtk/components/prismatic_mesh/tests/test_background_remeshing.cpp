#include <algorithm>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <wmtk/components/prismatic_mesh/background_remeshing.hpp>
#include <wmtk/components/prismatic_mesh/rod_mesh.hpp>

using namespace wmtk::components::prismatic_mesh;
using wmtk::MatrixXd;
using Tet = std::array<size_t, 4>;

namespace {
PrismaticMeshInput make_mesh(MatrixXd vertices, std::vector<Tet> tets)
{
    PrismaticMeshInput data;
    data.vertices = std::move(vertices);
    const size_t n = data.vertices.rows();
    data.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i) {
        if (!tet_volume_above_threshold(data.vertices, tets[i], 0))
            std::swap(tets[i][0], tets[i][1]);
        REQUIRE(tet_volume_above_threshold(data.vertices, tets[i], 0));
        for (int j = 0; j < 4; ++j) data.tetrahedra(i, j) = tets[i][j];
    }
    data.mesh = std::make_unique<wmtk::TetMesh>();
    data.mesh->init_with_isolated_vertices(n, tets);
    data.vertex_tags.assign(n, -1);
    data.source_vertex_ids.resize(n);
    for (size_t i = 0; i < n; ++i) data.source_vertex_ids[i] = 100 + i;
    data.corr_input_vid.assign(n, -1);
    data.corr_input_vertex.assign(n, -1);
    data.vertex_component_ids.assign(n, -1);
    data.singular_vertex_tags.assign(n, -1);
    data.input_cells.assign(tets.size(), 0);
    data.offset_tet_tags.assign(tets.size(), -1);
    data.input_to_offset_vertices.resize(n);
    data.input_to_components.resize(n);
    data.optimal_normals = MatrixXd::Zero(n, 3);
    data.target_positions = data.vertices;
    return data;
}

std::set<std::array<size_t, 3>> boundary(const PrismaticMeshInput& data)
{
    std::set<std::array<size_t, 3>> result;
    for (const auto& f : data.mesh->get_faces()) {
        if (!f.is_boundary_face(*data.mesh)) continue;
        const auto vertices = data.mesh->get_face_vertices(f);
        std::array<size_t, 3> ids;
        for (int i = 0; i < 3; ++i) ids[i] = vertices[i].vid(*data.mesh);
        std::sort(ids.begin(), ids.end());
        result.insert(ids);
    }
    return result;
}

void valid(const PrismaticMeshInput& data)
{
    REQUIRE(data.mesh->check_mesh_connectivity_validity());
    REQUIRE(data.tetrahedra.rows() == data.mesh->get_tets().size());
    REQUIRE(data.input_cells.size() == data.mesh->get_tets().size());
    REQUIRE(data.offset_tet_tags.size() == data.mesh->get_tets().size());
    for (const auto& t : data.mesh->get_tets()) {
        const auto tet = data.mesh->oriented_tet_vids(t);
        REQUIRE(tet_volume_above_threshold(data.vertices, tet, 0));
        for (int j = 0; j < 4; ++j) REQUIRE(data.tetrahedra(t.tid(*data.mesh), j) == tet[j]);
    }
}

PrismaticMeshInput bipyramid(bool three)
{
    const double h = three ? 2 : 0.05;
    MatrixXd v(5, 3);
    v << -1, 0, 0, 1, 0, 0, 0, 2, 0, 0, .5, h, 0, .5, -h;
    return make_mesh(
        v,
        three ? std::vector<Tet>{{3, 4, 0, 1}, {3, 4, 1, 2}, {3, 4, 2, 0}}
              : std::vector<Tet>{{0, 1, 2, 3}, {0, 2, 1, 4}});
}

PrismaticMeshInput blocked_octahedron()
{
    MatrixXd v(7, 3);
    v << 1, 0, 0, -2, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1, .1, 0, 0;
    std::vector<Tet> tets;
    for (size_t a : {0, 1})
        for (size_t b : {2, 3})
            for (size_t c : {4, 5}) tets.push_back({a, b, c, 6});
    auto data = make_mesh(v, tets);
    data.vertex_tags[1] = 1;
    data.input_vertices = {1};
    data.offset_vertices = {0, 2};
    OffsetComponent component;
    component.input_vertex = 1;
    component.vertices = {0, 2};
    component.singular = true; // no offset smoothing in the integration test
    data.offset_components.push_back(component);
    data.input_to_offset_vertices[1] = {0, 2};
    data.input_to_components[1] = {0};
    for (size_t a : {0, 2}) {
        data.vertex_tags[a] = 2;
        data.corr_input_vid[a] = 101;
        data.corr_input_vertex[a] = 1;
        data.vertex_component_ids[a] = 0;
        data.singular_vertex_tags[a] = 1;
    }
    return data;
}

BackgroundRemeshingOptions enabled()
{
    BackgroundRemeshingOptions options;
    options.enabled = true;
    options.quality_threshold = 1;
    return options;
}
} // namespace

TEST_CASE(
    "Rod background neighborhood remeshing preserves rod and far cells",
    "[rod][background_neighborhood]")
{
    MatrixXd v(12, 3);
    v << 1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1, .35, .05, .05, 2, 2, 2, 20, 0, 0,
        21, 0, 0, 20, 1, 0, 20, 0, 1;
    std::vector<Tet> tt;
    for (size_t a : {0, 1})
        for (size_t b : {2, 3})
            for (size_t c : {4, 5}) tt.push_back({a, b, c, 6});
    tt.push_back({7, 0, 2, 4});
    tt.push_back({8, 9, 10, 11});
    auto input = make_mesh(v, tt);
    input.offset_tet_tags[8] = 1;
    input.vertex_tags[7] = 1;
    input.input_vertices = {7};
    input.offset_vertices = {0, 2, 4};
    for (size_t u : input.offset_vertices) {
        input.vertex_tags[u] = 2;
        input.corr_input_vertex[u] = 7;
        input.corr_input_vid[u] = input.source_vertex_ids[7];
    }
    input.rod_state = std::make_shared<RodMeshState>();
    input.rod_state->input_vertices = {7};
    input.rod_state->radius = 1;
    auto& options = input.rod_state->options.background_remeshing;
    options.enabled = true;
    options.radius = 2;
    options.target_length = 2;
    options.passes = 1;
    options.time_limit = 10;
    const auto faces = boundary(input);
    const auto positions = input.vertices;
    const auto band = input.mesh->oriented_tet_vids(8);
    const auto far = input.mesh->oriented_tet_vids(9);
    remesh_rod_background(input);
    const auto& report = input.rod_state->report["background_neighborhood_remeshing"];
    REQUIRE(report["applied"] == true);
    CHECK(report["selected_background_tets"] == 8);
    CHECK(report["radius"] == 2);
    CHECK(report["rod_positions_and_connectivity_fixed"] == true);
    CHECK(boundary(input) == faces);
    valid(input);
    std::set<Tet> output;
    for (const auto& t : input.mesh->get_tets()) output.insert(input.mesh->oriented_tet_vids(t));
    CHECK(output.count(band) == 1);
    CHECK(output.count(far) == 1);
    for (size_t u : {0, 2, 4, 7, 8, 9, 10, 11}) CHECK(input.vertices.row(u) == positions.row(u));
    CHECK(input.vertex_tags.size() == input.vertices.rows());
    CHECK(input.corr_input_vid.size() == input.vertices.rows());
    const auto& counts = report["backend"]["accepted_topology_operations"];
    size_t accepted = 0;
    for (const auto& n : counts) accepted += n.get<size_t>();
    CHECK(accepted > 0);
}

TEST_CASE(
    "Coupled rod optimization uses remeshed background points and fixes the outer boundary",
    "[rod][background_coupled]")
{
    constexpr size_t n = 5;
    const auto vertex = [](size_t i, size_t j, size_t k) { return (i * n + j) * n + k; };
    MatrixXd p(n * n * n, 3);
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j)
            for (size_t k = 0; k < n; ++k)
                p.row(vertex(i, j, k)) << .5 * (int(i) - 2), .5 * (int(j) - 2), .5 * (int(k) - 2);
    std::vector<Tet> tt;
    for (size_t i = 0; i + 1 < n; ++i)
        for (size_t j = 0; j + 1 < n; ++j)
            for (size_t k = 0; k + 1 < n; ++k) {
                std::array<int, 3> order{0, 1, 2};
                do {
                    std::array<size_t, 3> xyz{i, j, k};
                    Tet t{vertex(i, j, k), 0, 0, 0};
                    for (int q = 0; q < 3; ++q) {
                        ++xyz[order[q]];
                        t[q + 1] = vertex(xyz[0], xyz[1], xyz[2]);
                    }
                    tt.push_back(t);
                } while (std::next_permutation(order.begin(), order.end()));
            }
    auto input = make_mesh(p, tt);
    const size_t center = vertex(2, 2, 2), other = vertex(2, 2, 1);
    std::vector<size_t> ring{vertex(3, 2, 2), vertex(3, 3, 2), vertex(3, 3, 3)};
    auto band = Tet{center, ring[0], ring[1], ring[2]};
    std::sort(band.begin(), band.end());
    for (size_t i = 0; i < tt.size(); ++i) {
        auto t = input.mesh->oriented_tet_vids(i);
        std::sort(t.begin(), t.end());
        if (t == band) input.offset_tet_tags[i] = 1;
    }
    for (size_t v : {center, other}) input.vertex_tags[v] = 1;
    input.input_vertices = {center, other};
    input.offset_vertices = ring;
    for (size_t v : ring) {
        input.vertex_tags[v] = 2;
        input.corr_input_vertex[v] = center;
        input.corr_input_vid[v] = input.source_vertex_ids[center];
    }
    input.rod_state = std::make_shared<RodMeshState>();
    auto& state = *input.rod_state;
    state.input_vertices = {center, other};
    state.mapping_locked = true;
    state.radius = .35;
    state.options.edge_based = true;
    state.options.coupled_rounds = 1;
    state.options.coupled_iterations = 4;
    auto& remesh = state.options.background_remeshing;
    remesh.enabled = true;
    remesh.radius = .6;
    remesh.target_length = .15;
    remesh.passes = 1;
    remesh.max_splits = 20;
    RodFrame frame;
    frame.center = center;
    frame.valid = true;
    frame.ring = ring;
    state.frames.push_back(frame);
    state.vertex_frame[center] = 0;
    state.mapping_controls.insert(center);
    state.input_edges.push_back({center, other});
    state.vertex_rotations[center] = state.vertex_rotations[other] = Eigen::Quaterniond::Identity();
    optimize_rod_background_coupled(input);
    const auto& report = state.report["coupled_optimization"];
    REQUIRE(report["rounds_completed"] == 1);
    const auto& geometry = report["rounds"][0]["geometry"];
    CHECK(geometry["movable_background_vertices"].get<size_t>() > 0);
    CHECK(geometry["new_background_vertices_available"].get<size_t>() > 0);
    CHECK(geometry["new_background_vertices_moved"].get<size_t>() > 0);
    CHECK(geometry["energy_after"][0].get<double>() < geometry["energy_before"][0].get<double>());
    CHECK(geometry["surface_energy_crosses_tet_prism_regions"] == true);
    CHECK(state.frames[0].ring == ring);
    for (size_t v = 0; v < size_t(p.rows()); ++v)
        if ((p.row(v).array().abs() == 1).any() || input.vertex_tags[v] == 1)
            CHECK(input.vertices.row(v) == p.row(v));
    bool found = false;
    for (const auto& cell : input.mesh->get_tets()) {
        auto t = input.mesh->oriented_tet_vids(cell);
        std::sort(t.begin(), t.end());
        found |= t == band && input.offset_tet_tags[cell.tid(*input.mesh)] == 1;
    }
    CHECK(found);
    valid(input);
}

TEST_CASE("background mean ratio is scale invariant with regular tet quality one")
{
    MatrixXd v(4, 3);
    v << 0, 0, 0, 1, 0, 0, .5, std::sqrt(3.) / 2, 0, .5, std::sqrt(3.) / 6, std::sqrt(2. / 3.);
    for (double scale : {1e-6, 1., 1e6})
        REQUIRE(background_tet_mean_ratio(v * scale, {0, 1, 2, 3}) == Catch::Approx(1));
    REQUIRE(background_tet_mean_ratio(v, {1, 0, 2, 3}) == 0);
}

TEST_CASE("background swaps improve shape and preserve cavity boundary and all coordinates")
{
    bool three = false;
    SECTION("two to three") {}
    SECTION("three to two")
    {
        three = true;
    }
    auto data = bipyramid(three);
    const auto vertices = data.vertices;
    const auto faces = boundary(data);
    const auto ids = data.source_vertex_ids;
    const auto report = remesh_background(data, 0, enabled());
    INFO(report.dump());
    REQUIRE(report[three ? "edge_swaps_3_to_2" : "face_swaps_2_to_3"].get<int>() == 1);
    REQUIRE(data.tetrahedra.rows() == (three ? 2 : 3));
    REQUIRE(data.vertices == vertices);
    REQUIRE(data.source_vertex_ids == ids);
    REQUIRE(boundary(data) == faces);
    REQUIRE(
        report["quality_after"]["minimum_mean_ratio"].get<double>() >
        report["quality_before"]["minimum_mean_ratio"].get<double>());
    REQUIRE(std::all_of(data.offset_tet_tags.begin(), data.offset_tet_tags.end(), [](int t) {
        return t == -1;
    }));
    valid(data);
}

TEST_CASE(
    "background remeshing never swaps across region interfaces and rolls back worsening swaps")
{
    auto data = bipyramid(false);
    auto options = enabled();
    SECTION("band interface")
    {
        data.offset_tet_tags[0] = 1;
    }
    SECTION("input interface")
    {
        data.input_cells[0] = 1;
    }
    SECTION("disabled")
    {
        options.enabled = false;
    }
    SECTION("no low quality cells")
    {
        options.quality_threshold = 1e-8;
    }
    SECTION("worsening swap")
    {
        data = bipyramid(true);
        data.vertices(3, 2) = .05;
        data.vertices(4, 2) = -.05;
    }
    SECTION("nonconvex cavity rejects inverted replacement tets")
    {
        data.vertices(3, 0) = 10;
        data.vertices(4, 0) = 10;
    }
    const auto v = data.vertices;
    const auto t = data.tetrahedra;
    const auto input = data.input_cells;
    const auto offset = data.offset_tet_tags;
    remesh_background(data, 0, options);
    REQUIRE(data.vertices == v);
    REQUIRE(data.tetrahedra == t);
    REQUIRE(data.input_cells == input);
    REQUIRE(data.offset_tet_tags == offset);
    valid(data);
}

TEST_CASE("background interior relaxation unlocks a genuinely blocked directed collapse")
{
    auto data = blocked_octahedron();
    const auto vertices = data.vertices;
    const auto faces = boundary(data);
    REQUIRE_FALSE(try_collapse_offset_edge(data, 0, 2, 0));
    const auto goals = background_blocked_collapses(data, 0);
    REQUIRE(std::any_of(goals.begin(), goals.end(), [](const auto& g) {
        return g.removed == 0 && g.survivor == 2;
    }));
    auto options = enabled();
    SECTION("quality trigger") {}
    SECTION("collapse trigger even with a tiny quality threshold")
    {
        options.quality_threshold = 1e-8;
    }
    const auto report = remesh_background(data, 0, options);
    INFO(report.dump());
    REQUIRE(report["vertex_moves"].get<int>() > 0);
    REQUIRE(data.vertices.topRows(6) == vertices.topRows(6));
    REQUIRE(data.vertices(6, 0) < 0);
    REQUIRE(boundary(data) == faces);
    valid(data);
    REQUIRE(try_collapse_offset_edge(data, 0, 2, 0));
    valid(data);
}

TEST_CASE("background cannot repair shell-volume failures and respects incomplete-star pins")
{
    auto data = blocked_octahedron();
    SECTION("shell blockers are excluded")
    {
        data.offset_tet_tags.assign(data.tetrahedra.rows(), 1);
        REQUIRE(background_blocked_collapses(data, 0).empty());
    }
    SECTION("an interior vertex shared with stored background stays fixed")
    {
        data.fixed_background_vertices.assign(7, false);
        data.fixed_background_vertices[6] = true;
        const auto v = data.vertices;
        remesh_background(data, 0, enabled());
        REQUIRE(data.vertices == v);
    }
}

TEST_CASE("background remeshing validates budgets and observes attempt limits")
{
    auto data = bipyramid(false);
    auto options = enabled();
    SECTION("invalid options")
    {
        options.quality_threshold = 0;
        REQUIRE_THROWS(remesh_background(data, 0, options));
        options = enabled();
        options.max_operations = 0;
        REQUIRE_THROWS(remesh_background(data, 0, options));
        options = enabled();
        options.passes = 0;
        REQUIRE_THROWS(remesh_background(data, 0, options));
    }
    SECTION("attempt limit")
    {
        options.max_attempts = 1;
        const auto report = remesh_background(data, 0, options);
        REQUIRE(report["attempts"] == 1);
        REQUIRE(report["budget_exhausted"] == true);
        valid(data);
    }
    SECTION("background must be retained")
    {
        OptimizationOptions optimization;
        optimization.background_remeshing = options;
        REQUIRE_THROWS(prism_main(data, .1, optimization));
    }
}

TEST_CASE("background diagnostics are exported for a pure tet output")
{
    struct Files
    {
        std::filesystem::path mesh =
            std::filesystem::temp_directory_path() /
            ("background_remesh_test_" +
             std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".vtu");
        std::filesystem::path report =
            mesh.parent_path() / (mesh.stem().string() + "_background_remeshing.json");
        ~Files()
        {
            std::error_code ec;
            std::filesystem::remove(mesh, ec);
            std::filesystem::remove(report, ec);
        }
    } files;
    auto data = bipyramid(false);
    data.background_remeshing_report = remesh_background(data, 0, enabled());
    write_prismatic_mesh(data, files.mesh);
    std::ifstream stream(files.report);
    REQUIRE(stream.good());
    nlohmann::json report;
    stream >> report;
    REQUIRE(report == data.background_remeshing_report);
    auto loaded = load_prismatic_mesh(files.mesh);
    REQUIRE(loaded.vertices == data.vertices);
    REQUIRE(loaded.tetrahedra == data.tetrahedra);
    REQUIRE(loaded.source_vertex_ids == data.source_vertex_ids);
}
