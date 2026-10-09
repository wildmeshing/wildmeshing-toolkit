#include <algorithm>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <wmtk/components/prismatic_mesh/background_remeshing.hpp>

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
