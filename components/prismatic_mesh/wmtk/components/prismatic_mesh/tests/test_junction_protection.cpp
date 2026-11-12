#include <Eigen/Geometry>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/components/prismatic_mesh/junction_protection.hpp>

using namespace wmtk::components::prismatic_mesh;
namespace {
using Tri = std::array<size_t, 3>;
using Tet = std::array<size_t, 4>;

PrismaticMeshInput surface_fixture(const std::vector<Tri>& faces, bool rod = false)
{
    PrismaticMeshInput input;
    constexpr size_t nv = 8;
    input.vertices.resize(nv + faces.size() + (rod ? 2 : 0), 3);
    input.vertices.topRows(nv) << 0, 0, 0, 1, 0, 0, 0, 1, 0, -1, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0, -1,
        1, 0, 1;
    const size_t n = input.vertices.rows();
    input.vertex_tags.assign(n, 1);
    input.source_vertex_ids.resize(n);
    input.corr_input_vid.assign(n, -1);
    input.corr_input_vertex.assign(n, -1);
    input.input_to_offset_vertices.resize(n);
    std::vector<Tet> tets;
    for (size_t i = 0; i < faces.size(); ++i) {
        const auto f = faces[i];
        const wmtk::Vector3d a = input.vertices.row(f[0]);
        const wmtk::Vector3d b = input.vertices.row(f[1]);
        const wmtk::Vector3d c = input.vertices.row(f[2]);
        input.vertices.row(nv + i) =
            ((a + b + c) / 3 + .25 * (b - a).cross(c - a).normalized()).transpose();
        input.vertex_tags[nv + i] = 2;
        input.corr_input_vertex[nv + i] = 0;
        tets.push_back({f[0], f[1], f[2], nv + i});
    }
    if (rod) {
        input.vertices.row(n - 2) << .2, .1, .25;
        input.vertices.row(n - 1) << -.1, .2, .75;
        input.vertex_tags[n - 2] = input.vertex_tags[n - 1] = 2;
        input.corr_input_vertex[n - 2] = 0;
        input.corr_input_vertex[n - 1] = 5;
        tets.push_back({0, 5, n - 2, n - 1});
    }
    input.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i) {
        if (!tet_volume_above_threshold(input.vertices, tets[i], 0))
            std::swap(tets[i][0], tets[i][1]);
        REQUIRE(tet_volume_above_threshold(input.vertices, tets[i], 0));
        for (size_t j = 0; j < 4; ++j) input.tetrahedra(i, j) = tets[i][j];
    }
    for (size_t v = 0; v < n; ++v) {
        input.source_vertex_ids[v] = 100 + v;
        if (input.vertex_tags[v] == 1)
            input.input_vertices.push_back(v);
        else {
            input.offset_vertices.push_back(v);
            input.corr_input_vid[v] = 100 + input.corr_input_vertex[v];
            input.input_to_offset_vertices[input.corr_input_vertex[v]].push_back(v);
        }
    }
    input.input_cells.assign(tets.size(), 0);
    input.offset_tet_tags.assign(tets.size(), 1);
    input.mesh = std::make_unique<wmtk::TetMesh>();
    input.mesh->init_with_isolated_vertices(n, tets);
    return input;
}

std::vector<Tri> interior_disk()
{
    return {{0, 1, 2}, {0, 2, 3}, {0, 3, 4}, {0, 4, 1}};
}
} // namespace

TEST_CASE("an ordinary X intersection is not a protected terminal connection")
{
    auto faces = interior_disk();
    for (Tri f : {Tri{0, 1, 5}, Tri{0, 5, 3}, Tri{0, 3, 6}, Tri{0, 6, 1}}) faces.push_back(f);
    auto input = surface_fixture(faces);
    initialize_junction_protection(input);
    REQUIRE(input.junction_protection->centers.empty());
    REQUIRE(input.junction_protection->offset_vertices.empty());
    for (size_t v : input.offset_vertices) REQUIRE(junction_collapse_allowed(input, v));
}

TEST_CASE("shell curve ends on open boundary do not protect the intersection line")
{
    auto faces = interior_disk();
    faces.push_back({0, 1, 5});
    auto input = surface_fixture(faces);
    initialize_junction_protection(input);
    REQUIRE(input.junction_protection->shell_curve_end_vertices == std::set<size_t>{0, 1});
    REQUIRE(input.junction_protection->open_boundary_vertices.count(0));
    REQUIRE(input.junction_protection->open_boundary_vertices.count(1));
    REQUIRE(input.junction_protection->shell_terminal_vertices.empty());
    REQUIRE(input.junction_protection->centers.empty());
    for (size_t v : input.offset_vertices) REQUIRE(junction_collapse_allowed(input, v));
}

TEST_CASE("only a shell curve endpoint off open boundary protects its one-ring")
{
    // Two triangular cone fans share edge (0,5). The intersection ends at 0
    // and 5, but only 5 belongs to the fans' outer open-boundary loops.
    auto input =
        surface_fixture({{0, 5, 1}, {0, 1, 2}, {0, 2, 5}, {0, 5, 3}, {0, 3, 4}, {0, 4, 5}});
    initialize_junction_protection(input);
    REQUIRE(input.junction_protection->shell_curve_end_vertices == std::set<size_t>{0, 5});
    REQUIRE_FALSE(input.junction_protection->open_boundary_vertices.count(0));
    REQUIRE(input.junction_protection->open_boundary_vertices.count(5));
    REQUIRE(input.junction_protection->shell_terminal_vertices == std::set<size_t>{0});
    REQUIRE(input.junction_protection->original_band_cells.size() == 6);
    for (size_t v : input.offset_vertices) REQUIRE_FALSE(junction_collapse_allowed(input, v));
    REQUIRE_NOTHROW(verify_junction_one_rings(input));
    input.vertex_component_ids.assign(input.vertices.rows(), 0);
    input.singular_vertex_tags.assign(input.vertices.rows(), 0);
    input.target_positions = wmtk::MatrixXd::Zero(input.vertices.rows(), 3);
    apply_junction_targets(input);
    for (size_t v : input.junction_protection->offset_vertices) {
        REQUIRE(input.target_positions.row(v) == input.vertices.row(v));
        REQUIRE(input.singular_vertex_tags[v] == 1);
    }
    input.tetrahedra.conservativeResize(input.tetrahedra.rows() - 1, 4);
    REQUIRE_THROWS(verify_junction_one_rings(input));
}

TEST_CASE("two shell boundaries meeting at their corners do not create an interior attachment")
{
    auto input = surface_fixture({{0, 1, 2}, {0, 5, 7}});
    initialize_junction_protection(input);
    REQUIRE(input.junction_protection->centers.empty());
}

TEST_CASE("a free rod attached to a shell protects the shared point one-ring")
{
    auto faces = interior_disk();
    SECTION("attachment in the shell interior") {}
    SECTION("attachment on shell open boundary")
    {
        faces = {{0, 1, 2}};
    }
    auto input = surface_fixture(faces, true);
    initialize_junction_protection(input);
    REQUIRE(input.junction_protection->rod_shell_vertices == std::set<size_t>{0});
    REQUIRE(input.junction_protection->shell_terminal_vertices.empty());
    REQUIRE(input.junction_protection->rod_end_vertices == std::set<size_t>{0, 5});
    REQUIRE(input.junction_protection->centers == std::set<size_t>{0});
    REQUIRE(input.junction_protection->rod_free_end_vertices == std::set<size_t>{5});
    for (size_t v : input.offset_vertices) REQUIRE_FALSE(junction_collapse_allowed(input, v));
    REQUIRE_NOTHROW(verify_junction_one_rings(input));
}

TEST_CASE("free rod endpoints do not protect one-rings")
{
    auto input = surface_fixture({}, true);
    initialize_junction_protection(input);
    REQUIRE(input.junction_protection->rod_shell_vertices.empty());
    REQUIRE(input.junction_protection->rod_end_vertices == std::set<size_t>{0, 5});
    REQUIRE(input.junction_protection->rod_free_end_vertices == std::set<size_t>{0, 5});
    REQUIRE(input.junction_protection->centers.empty());
    REQUIRE(input.junction_protection->original_band_cells.empty());
    for (size_t v : input.offset_vertices) REQUIRE(junction_collapse_allowed(input, v));
    REQUIRE_NOTHROW(verify_junction_one_rings(input));
}

TEST_CASE("branching rod joints retain one-ring protection while their free ends do not")
{
    auto input = surface_fixture({}, true);
    const size_t n = input.vertices.rows();
    input.tetrahedra.conservativeResize(3, 4);
    for (int i = 1; i < 3; ++i) {
        Tet t{0, static_cast<size_t>(i), n - 2, n - 1};
        if (!tet_volume_above_threshold(input.vertices, t, 0)) std::swap(t[0], t[1]);
        REQUIRE(tet_volume_above_threshold(input.vertices, t, 0));
        for (int j = 0; j < 4; ++j) input.tetrahedra(i, j) = t[j];
    }
    input.input_cells.resize(3, 0);
    input.offset_tet_tags.resize(3, 1);
    initialize_junction_protection(input);
    REQUIRE(input.junction_protection->rod_joint_vertices == std::set<size_t>{0});
    REQUIRE(input.junction_protection->rod_free_end_vertices == std::set<size_t>{1, 2, 5});
    REQUIRE(input.junction_protection->centers == std::set<size_t>{0});
    REQUIRE(input.junction_protection->original_band_cells.size() == 3);
}

TEST_CASE("transition envelope smoothing fills a dent without changing protected topology")
{
    PrismaticMeshInput input;
    input.vertices.resize(8, 3);
    // A tetrahedral star of an octahedron, with one dented face-center vertex.
    input.vertices << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, -1, 0, 0, 0, -1, 0, 0, 0, -1, .2, .2, .2;
    const std::vector<Tri> surface = {
        {1, 2, 7},
        {2, 3, 7},
        {3, 1, 7},
        {1, 3, 5},
        {1, 5, 6},
        {1, 6, 2},
        {4, 3, 2},
        {4, 5, 3},
        {4, 6, 5},
        {4, 2, 6}};
    input.tetrahedra.resize(surface.size(), 4);
    input.vertex_tags.assign(8, 2);
    input.vertex_tags[0] = 1;
    input.source_vertex_ids = {100, 101, 102, 103, 104, 105, 106, 107};
    input.corr_input_vertex.assign(8, 0);
    input.corr_input_vertex[0] = -1;
    input.input_cells.assign(surface.size(), 0);
    input.offset_tet_tags.assign(surface.size(), 1);
    input.junction_protection = std::make_shared<JunctionProtection>();
    input.junction_protection->offset_vertices.insert(7);
    PrismDominantMesh hybrid;
    for (size_t i = 0; i < surface.size(); ++i) {
        Tet t = {0, surface[i][0], surface[i][1], surface[i][2]};
        if (!tet_volume_above_threshold(input.vertices, t, 0)) std::swap(t[1], t[2]);
        for (int j = 0; j < 4; ++j) input.tetrahedra(i, j) = t[j];
        hybrid.cells.push_back({HybridCellType::Tetrahedron, {t.begin(), t.end()}, {i}});
        hybrid.source_tet_to_cell.push_back(i);
    }
    const auto original = input.vertices.eval();
    const auto topology = input.tetrahedra.eval();
    SECTION("disabled stage leaves all coordinates untouched")
    {
        smooth_transition_regions(input, hybrid, 0, 0);
        REQUIRE(input.vertices == original);
    }
    SECTION("protected geometry may relax while connectivity stays protected")
    {
        smooth_transition_regions(input, hybrid, 0, 8);
        REQUIRE(input.tetrahedra == topology);
        REQUIRE(input.vertices.row(0) == original.row(0));
        REQUIRE(input.vertices.middleRows(4, 3) == original.middleRows(4, 3));
        for (int j = 0; j < 3; ++j) {
            REQUIRE(input.vertices(7, j) > .33);
            REQUIRE(input.vertices(7, j) < .34);
        }
        REQUIRE(
            hybrid.report["transition_smoothing"]["final_concavity_energy"].get<double>() <
            hybrid.report["transition_smoothing"]["initial_concavity_energy"].get<double>());
        REQUIRE_FALSE(junction_collapse_allowed(input, 7));
        REQUIRE(hybrid.report["transition_smoothing"]["protected_vertices_moved"] == 1);
        REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
    }
    SECTION("a volume floor prevents a geometrically invalid relaxation")
    {
        // Above the volume of every affected tet: no move is admissible.
        smooth_transition_regions(input, hybrid, 1, 8);
        REQUIRE(input.vertices == original);
        REQUIRE(hybrid.report["transition_smoothing"]["accepted_moves"] == 0);
        REQUIRE(hybrid.report["transition_smoothing"]["blocked_moves"].get<size_t>() > 0);
    }
}
