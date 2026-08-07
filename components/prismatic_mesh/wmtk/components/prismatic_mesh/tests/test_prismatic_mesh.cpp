#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/prismatic_mesh/OffsetOperations.hpp>
#include <wmtk/components/prismatic_mesh/Recovery.hpp>
#include <wmtk/components/prismatic_mesh/Types.hpp>
#include <wmtk/utils/io.hpp>

#include <cmath>
#include <filesystem>
#include <map>

using namespace wmtk;
using namespace wmtk::components::prismatic_mesh;

namespace {

PrismaticMeshInput one_tet_input()
{
    PrismaticMeshInput input;
    input.vertices.resize(4, 3);
    input.vertices << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    input.tets = {{{0, 1, 2, 3}}};
    input.tet_groups = {{"solid"}};
    return input;
}

PrismaticMeshInput embedded_shell_input()
{
    PrismaticMeshInput input;
    input.vertices.resize(5, 3);
    input.vertices << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, -1;
    input.tets = {{{0, 1, 2, 3}}, {{0, 2, 1, 4}}};
    input.tet_groups = {{"ambient"}, {"ambient"}};
    input.face_groups["shell"] = {{{0, 1, 2}}};
    return input;
}

using Facet = std::vector<size_t>;

std::map<Facet, size_t> facet_incidence(const HybridVolumeMesh& mesh)
{
    std::map<Facet, size_t> result;
    const auto add = [&result](Facet facet) {
        std::sort(facet.begin(), facet.end());
        ++result[std::move(facet)];
    };
    for (const auto& t : mesh.tets) {
        add({t[1], t[2], t[3]});
        add({t[0], t[3], t[2]});
        add({t[0], t[1], t[3]});
        add({t[0], t[2], t[1]});
    }
    for (const auto& p : mesh.prisms) {
        add({p[0], p[1], p[2]});
        add({p[3], p[4], p[5]});
        add({p[0], p[1], p[4], p[3]});
        add({p[1], p[2], p[5], p[4]});
        add({p[2], p[0], p[3], p[5]});
    }
    for (const auto& p : mesh.pyramids) {
        add({p[0], p[1], p[2], p[3]});
        add({p[0], p[1], p[4]});
        add({p[1], p[2], p[4]});
        add({p[2], p[3], p[4]});
        add({p[3], p[0], p[4]});
    }
    return result;
}

void initialize_collapse_fixture(PrismOffsetTetMesh& mesh)
{
    mesh.init(
        6,
        {{{0, 3, 2, 4}},
         {{1, 2, 3, 4}},
         {{0, 1, 2, 3}},
         {{0, 1, 3, 5}},
         {{1, 3, 5, 4}},
         {{0, 3, 4, 5}}});
    const std::array<Vector3d, 6> positions = {
        Vector3d(0, 0, -1),
        Vector3d(0, 0, 1),
        Vector3d(1, 1, 0),
        Vector3d(1, 0, 0),
        Vector3d(2, 0, 0),
        Vector3d(1, -1, 0)};
    for (size_t i = 0; i < positions.size(); ++i) {
        mesh.m_vertex_attribute[i].m_posf = positions[i];
        mesh.m_vertex_attribute[i].label = 2;
        mesh.m_vertex_attribute[i].source_vid = static_cast<int64_t>(i);
    }
    for (const auto& tet : mesh.get_tets()) {
        mesh.m_tet_attribute[tet.tid(mesh)].label = 0;
    }
}

std::vector<std::array<size_t, 2>> initialize_split_fixture(PrismOffsetTetMesh& mesh)
{
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 2, 1, 4}}, {{0, 1, 3, 4}}});
    const std::array<Vector3d, 5> positions = {
        Vector3d(0, 0, 0),
        Vector3d(1, 0, 0),
        Vector3d(0, 1, 0),
        Vector3d(0, 0, 1),
        Vector3d(0.2, -0.2, -1)};
    for (size_t i = 0; i < positions.size(); ++i) {
        mesh.m_vertex_attribute[i].m_posf = positions[i];
        mesh.m_vertex_attribute[i].source_vid = static_cast<int64_t>(i);
    }
    std::vector<PrismOffsetTetMesh::Tuple> new_edges;
    REQUIRE(mesh.split_edge(mesh.tuple_from_edge(0, 0), new_edges));
    std::vector<std::array<size_t, 2>> result;
    for (const auto& edge : new_edges) {
        auto vertices = std::array<size_t, 2>{edge.vid(mesh), edge.switch_vertex(mesh).vid(mesh)};
        std::sort(vertices.begin(), vertices.end());
        result.push_back(vertices);
    }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

} // namespace

TEST_CASE("paper equal-source collapse uses current TetMesh rollback", "[prismatic_mesh][offset]")
{
    wmtk::components::topological_offset::Parameters parameters;
    PrismOffsetTetMesh seed(parameters);
    const auto edges = initialize_split_fixture(seed);

    bool accepted = false;
    for (const auto& edge : edges) {
        PrismOffsetTetMesh mesh(parameters);
        initialize_split_fixture(mesh);
        mesh.m_vertex_attribute[edge[1]].source_vid = mesh.m_vertex_attribute[edge[0]].source_vid;
        mesh.m_vertex_attribute[edge[0]].label = 2;
        mesh.m_vertex_attribute[edge[1]].label = 2;
        for (const auto& tet : mesh.get_incident_tets_for_edge(edge[0], edge[1])) {
            mesh.m_tet_attribute[tet.tid(mesh)].label = 2;
        }
        if (mesh.count_equal_source_edges() == 0) continue;
        OffsetOperationStats stats;
        if (mesh.collapse_equal_source_edges(&stats) == 0) continue;
        CHECK(stats.equal_source_attempts >= 1);
        CHECK(stats.equal_source_collapses == 1);
        CHECK(mesh.check_mesh_connectivity_validity());
        CHECK(mesh.tet_size() < seed.tet_size());
        accepted = true;
        break;
    }
    CHECK(accepted);
}

TEST_CASE("paper equal-source collapse rejects a link obstruction", "[prismatic_mesh][offset]")
{
    wmtk::components::topological_offset::Parameters parameters;
    PrismOffsetTetMesh mesh(parameters);
    mesh.init(4, {{{0, 1, 2, 3}}});
    const std::array<Vector3d, 4> positions = {
        Vector3d(0, 0, 0),
        Vector3d(1, 0, 0),
        Vector3d(0, 1, 0),
        Vector3d(0, 0, 1)};
    for (size_t i = 0; i < positions.size(); ++i) {
        mesh.m_vertex_attribute[i].m_posf = positions[i];
        mesh.m_vertex_attribute[i].label = 2;
        mesh.m_vertex_attribute[i].source_vid = static_cast<int64_t>(i);
    }
    mesh.m_vertex_attribute[1].source_vid = 0;
    mesh.m_tet_attribute[0].label = 2;

    OffsetOperationStats stats;
    REQUIRE(mesh.count_equal_source_edges() == 1);
    CHECK(mesh.collapse_equal_source_edges(&stats) == 0);
    CHECK(stats.equal_source_attempts == 1);
    CHECK(stats.equal_source_collapses == 0);
    CHECK(mesh.vertex_size() == 4);
    CHECK(mesh.tet_size() == 1);
    CHECK(mesh.check_mesh_connectivity_validity());
}

TEST_CASE("paper tau22 unlock rolls back both blocked diagonals", "[prismatic_mesh][offset]")
{
    wmtk::components::topological_offset::Parameters parameters;
    PrismOffsetTetMesh mesh(parameters);
    initialize_collapse_fixture(mesh);
    mesh.m_vertex_attribute[0].label = 1;
    mesh.m_vertex_attribute[0].source_vid = 0;
    mesh.m_vertex_attribute[1].label = 1;
    mesh.m_vertex_attribute[1].source_vid = 1;
    mesh.m_vertex_attribute[2].label = 2;
    mesh.m_vertex_attribute[2].source_vid = 0;
    mesh.m_vertex_attribute[3].label = 2;
    mesh.m_vertex_attribute[3].source_vid = 1;
    mesh.m_vertex_attribute[4].label = 0;
    mesh.m_vertex_attribute[5].label = 0;
    mesh.m_tet_attribute[2].label = 2;

    REQUIRE(mesh.count_tau_2_2() == 1);
    const size_t vertices_before = mesh.vertex_size();
    const size_t tets_before = mesh.tet_size();
    OffsetOperationStats stats;
    CHECK(mesh.unlock_tau_2_2(&stats) == 0);
    CHECK(stats.unlock_attempts == 1);
    CHECK(stats.unlocks == 0);
    CHECK(stats.unlock_rollbacks == 2);
    CHECK(mesh.vertex_size() == vertices_before);
    CHECK(mesh.tet_size() == tets_before);
    CHECK(mesh.count_tau_2_2() == 1);
    CHECK(mesh.check_mesh_connectivity_validity());
}

TEST_CASE(
    "paper open-boundary tau classification removes only incompatible tets",
    "[prismatic_mesh][offset]")
{
    wmtk::components::topological_offset::Parameters parameters;
    const std::vector<std::array<size_t, 3>> faces = {{{0, 1, 2}}};

    SECTION("tau 2 1 is removed")
    {
        PrismOffsetTetMesh mesh(parameters);
        mesh.init(4, {{{0, 1, 2, 3}}});
        mesh.m_tet_attribute[0].label = 2;
        mesh.m_vertex_attribute[0].label = 1;
        mesh.m_vertex_attribute[0].source_vid = 0;
        mesh.m_vertex_attribute[1].label = 1;
        mesh.m_vertex_attribute[1].source_vid = 1;
        mesh.m_vertex_attribute[2].label = 2;
        mesh.m_vertex_attribute[2].source_vid = 0;
        mesh.m_vertex_attribute[3].label = 2;
        mesh.m_vertex_attribute[3].source_vid = 0;

        const auto stats = mesh.handle_open_boundaries(faces);
        CHECK(stats.boundary_edges == 3);
        CHECK(stats.candidate_tets == 1);
        CHECK(stats.removed_tets == 1);
        CHECK(mesh.tet_size() == 0);
    }

    SECTION("tau 3 3 is preserved")
    {
        PrismOffsetTetMesh mesh(parameters);
        mesh.init(4, {{{0, 1, 2, 3}}});
        mesh.m_tet_attribute[0].label = 2;
        mesh.m_vertex_attribute[0].label = 1;
        mesh.m_vertex_attribute[0].source_vid = 0;
        for (size_t i = 1; i < 4; ++i) {
            mesh.m_vertex_attribute[i].label = 2;
            mesh.m_vertex_attribute[i].source_vid = static_cast<int64_t>(i - 1);
        }

        const auto stats = mesh.handle_open_boundaries(faces);
        CHECK(stats.boundary_edges == 3);
        CHECK(stats.candidate_tets == 1);
        CHECK(stats.removed_tets == 0);
        CHECK(mesh.tet_size() == 1);
    }

    SECTION("closed annotated surfaces have no open boundary")
    {
        PrismOffsetTetMesh mesh(parameters);
        mesh.init(4, {{{0, 1, 2, 3}}});
        mesh.m_tet_attribute[0].label = 2;
        for (size_t i = 0; i < 4; ++i) {
            mesh.m_vertex_attribute[i].label = i == 0 ? 1 : 2;
            mesh.m_vertex_attribute[i].source_vid = static_cast<int64_t>(i);
        }
        const std::vector<std::array<size_t, 3>> closed = {
            {{0, 1, 2}},
            {{0, 3, 1}},
            {{1, 3, 2}},
            {{0, 2, 3}}};
        const auto stats = mesh.handle_open_boundaries(closed);
        CHECK(stats.boundary_edges == 0);
        CHECK(stats.candidate_tets == 0);
        CHECK(stats.removed_tets == 0);
        CHECK(mesh.tet_size() == 1);
    }
}

TEST_CASE("cross section validation", "[prismatic_mesh]")
{
    const std::vector<Vector2d> square = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    CHECK(is_valid_cross_section(square));

    auto reversed = square;
    std::reverse(reversed.begin(), reversed.end());
    CHECK_FALSE(is_valid_cross_section(reversed));

    const std::vector<Vector2d> bowtie = {{-1, -1}, {1, 1}, {-1, 1}, {1, -1}};
    CHECK_FALSE(is_valid_cross_section(bowtie));
}

TEST_CASE("annotated simplices must belong to the tetrahedral complex", "[prismatic_mesh][input]")
{
    auto bad_face = one_tet_input();
    bad_face.vertices.conservativeResize(5, 3);
    bad_face.vertices.row(4) = Vector3d(2, 2, 2);
    bad_face.face_groups["shell"] = {{{0, 1, 4}}};
    PrismaticMeshParameters shell_parameters;
    shell_parameters.shells = {{"shell", 0.1}};
    CHECK_THROWS(generate_prismatic_mesh(bad_face, shell_parameters));

    auto duplicate_edge = one_tet_input();
    duplicate_edge.edge_groups["rod"] = {{{0, 1}}, {{1, 0}}};
    RodDefinition rod;
    rod.group = "rod";
    rod.thickness = 0.1;
    rod.cross_section = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    PrismaticMeshParameters rod_parameters;
    rod_parameters.rods = {rod};
    CHECK_THROWS(generate_prismatic_mesh(duplicate_edge, rod_parameters));
}

TEST_CASE("hybrid cell validity", "[prismatic_mesh]")
{
    const std::array<Vector3d, 6> prism = {
        Vector3d(0, 0, 0),
        Vector3d(1, 0, 0),
        Vector3d(0, 1, 0),
        Vector3d(0, 0, 1),
        Vector3d(1, 0, 1),
        Vector3d(0, 1, 1)};
    CHECK(is_valid_prism(prism));

    auto inverted_prism = prism;
    std::swap(inverted_prism[4], inverted_prism[5]);
    CHECK_FALSE(is_valid_prism(inverted_prism));

    const std::array<Vector3d, 5> pyramid = {
        Vector3d(0, 0, 0),
        Vector3d(1, 0, 0),
        Vector3d(1, 1, 0),
        Vector3d(0, 1, 0),
        Vector3d(0.5, 0.5, 1)};
    CHECK(is_valid_pyramid(pyramid));
}

TEST_CASE("paper marked-vertex recovery partitions", "[prismatic_mesh][recovery]")
{
    const std::array<size_t, 6> prism = {{0, 1, 2, 3, 4, 5}};
    const std::array<Vector3d, 6> points = {
        Vector3d(0, 0, 0),
        Vector3d(1, 0, 0),
        Vector3d(0, 1, 0),
        Vector3d(0, 0, 1),
        Vector3d(1, 0, 1),
        Vector3d(0, 1, 1)};

    const auto unmarked = recover_prism_partition(prism, {{false, false, false}});
    CHECK(unmarked.prisms.size() == 1);
    CHECK(unmarked.pyramids.empty());
    CHECK(unmarked.tets.empty());

    for (size_t marked_vertex = 0; marked_vertex < 3; ++marked_vertex) {
        std::array<bool, 3> marked = {{false, false, false}};
        marked[marked_vertex] = true;
        const auto recovered = recover_prism_partition(prism, marked);
        CHECK(recovered.prisms.empty());
        CHECK(recovered.pyramids.size() == 1);
        CHECK(recovered.tets.size() == 1);
        std::array<Vector3d, 5> pyramid_points;
        for (size_t i = 0; i < 5; ++i) {
            pyramid_points[i] = points[recovered.pyramids.front()[i]];
        }
        std::array<Vector3d, 4> tet_points;
        for (size_t i = 0; i < 4; ++i) tet_points[i] = points[recovered.tets.front()[i]];
        CHECK(is_valid_pyramid(pyramid_points));
        CHECK(is_valid_tet(tet_points));
    }

    for (const auto& marked :
         {std::array<bool, 3>{{true, true, false}},
          std::array<bool, 3>{{true, false, true}},
          std::array<bool, 3>{{false, true, true}},
          std::array<bool, 3>{{true, true, true}}}) {
        const auto recovered = recover_prism_partition(prism, marked);
        CHECK(recovered.prisms.empty());
        CHECK(recovered.pyramids.empty());
        CHECK(recovered.tets.size() == 3);
    }
}

TEST_CASE("paper shell layer", "[prismatic_mesh]")
{
    const auto input = embedded_shell_input();

    PrismaticMeshParameters parameters;
    parameters.shells.push_back({"shell", 0.1});
    const auto result = generate_prismatic_mesh(input, parameters);

    // One side is an exact prism column. The other contains an embedding
    // subdivision and must conservatively remain tetrahedral rather than
    // overlap a nominal prism candidate.
    CHECK(result.mesh.prisms.size() == 1);
    CHECK_FALSE(result.mesh.tets.empty());
    CHECK(result.report.shell_prisms == 1);
    CHECK(result.report.topological_offset_runs == 1);
    CHECK(result.report.open_boundary_edges == 3);
    CHECK(result.mesh.vertices.rows() == 9);
    for (const auto& [_, count] : facet_incidence(result.mesh)) CHECK(count <= 2);
}

TEST_CASE("solid preservation", "[prismatic_mesh]")
{
    const auto input = one_tet_input();
    PrismaticMeshParameters parameters;
    parameters.solid_groups = {"solid"};
    const auto result = generate_prismatic_mesh(input, parameters);

    REQUIRE(result.mesh.tets.size() == 1);
    CHECK(result.mesh.tets[0] == input.tets[0]);
    CHECK(result.report.solid_tets == 1);
}

TEST_CASE("shell layer respects an annotated solid side", "[prismatic_mesh]")
{
    auto input = embedded_shell_input();
    input.tet_groups[1] = {"solid"};

    PrismaticMeshParameters parameters;
    parameters.solid_groups = {"solid"};
    parameters.shells.push_back({"shell", 0.1});
    const auto result = generate_prismatic_mesh(input, parameters);

    REQUIRE(result.mesh.tets.size() == 1);
    REQUIRE(result.mesh.prisms.size() == 1);
    CHECK(result.report.solid_blocked_shell_sides == 1);
    CHECK(result.mesh.vertices.rows() == 7);

    const auto facets = facet_incidence(result.mesh);
    CHECK(facets.at({0, 1, 2}) == 2);
    for (const auto& [_, count] : facets) CHECK(count <= 2);
}

TEST_CASE("shell sweep is clipped by a nearby annotated sheet", "[prismatic_mesh]")
{
    PrismaticMeshInput input;
    input.vertices.resize(6, 3);
    input.vertices << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0.1, 1, 0, 0.1, 0, 1, 0.1;
    input.tets = {{{0, 1, 2, 3}}, {{3, 5, 4, 0}}};
    input.tet_groups = {{"ambient"}, {"ambient"}};
    input.face_groups["lower"] = {{{0, 1, 2}}};
    input.face_groups["upper"] = {{{3, 4, 5}}};

    PrismaticMeshParameters parameters;
    parameters.use_topological_offset = false;
    parameters.shells = {{"lower", 0.2}, {"upper", 0.2}};
    const auto result = generate_prismatic_mesh(input, parameters);

    CHECK(result.report.collision_clipped_shell_vertices == 6);
    CHECK(std::abs(result.report.minimum_shell_scale - 0.4) < 1e-12);
    CHECK(result.mesh.prisms.size() == 4);
    CHECK(result.report.fallback_tets == 0);
}

TEST_CASE("nonmanifold shell edge is partitioned into conforming face fans", "[prismatic_mesh]")
{
    PrismaticMeshInput input;
    input.vertices.resize(10, 3);
    input.vertices << -1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, -1, 0, 0, 1, 0,
        1, -1, 0, 1, 1;
    input.face_groups["shell"] = {
        {{0, 1, 4}},
        {{0, 4, 3}},
        {{1, 2, 5}},
        {{1, 5, 4}},
        {{6, 1, 4}},
        {{6, 4, 8}},
        {{1, 7, 9}},
        {{1, 9, 4}}};

    PrismaticMeshParameters parameters;
    parameters.use_topological_offset = false;
    parameters.shells = {{"shell", 0.05}};
    const auto result = generate_prismatic_mesh(input, parameters);

    CHECK(result.report.nonmanifold_shell_edges == 1);
    CHECK(result.report.shell_vertex_sectors > 10);
    CHECK(result.mesh.prisms.size() == 16);
    for (const auto& [_, count] : facet_incidence(result.mesh)) CHECK(count <= 2);
}

TEST_CASE("straight polygonal rod", "[prismatic_mesh]")
{
    auto input = one_tet_input();
    input.edge_groups["rod"] = {{{0, 3}}};

    RodDefinition rod;
    rod.group = "rod";
    rod.thickness = 0.05;
    rod.cross_section = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};

    PrismaticMeshParameters parameters;
    parameters.rods.push_back(rod);
    const auto result = generate_prismatic_mesh(input, parameters);
    CHECK(result.mesh.prisms.size() == 4);
    CHECK(result.report.rod_prisms == 4);
}

TEST_CASE("straight rods support arbitrary star-shaped profiles", "[prismatic_mesh][rod]")
{
    auto input = one_tet_input();
    input.edge_groups["rod"] = {{{0, 3}}};

    const std::vector<std::vector<Vector2d>> profiles = {
        {{1, 0},
         {0.35, 0.35},
         {0, 1},
         {-0.35, 0.35},
         {-1, 0},
         {-0.35, -0.35},
         {0, -1},
         {0.35, -0.35}},
        {{-1, -0.3},
         {-0.3, -0.3},
         {-0.3, -1.6},
         {0.3, -1.6},
         {0.3, -0.3},
         {1, -0.3},
         {1, 0.4},
         {-1, 0.4}}};

    for (const auto& profile : profiles) {
        INFO("profile with " << profile.size() << " vertices");
        REQUIRE(is_valid_cross_section(profile));
        RodDefinition rod;
        rod.group = "rod";
        rod.thickness = 0.05;
        rod.cross_section = profile;
        PrismaticMeshParameters parameters;
        parameters.rods.push_back(rod);
        const auto result = generate_prismatic_mesh(input, parameters);
        CHECK(result.mesh.prisms.size() == profile.size());
        CHECK(result.report.fallback_tets == 0);
    }
}

TEST_CASE("invalid rod segments are refined locally", "[prismatic_mesh]")
{
    auto input = one_tet_input();
    input.vertices.row(1) = Vector3d(1, 0, 0);
    input.vertices.row(2) = Vector3d(1, 1, 0);
    input.edge_groups["rod"] = {{{0, 1}}, {{1, 2}}};

    RodDefinition rod;
    rod.group = "rod";
    rod.thickness = 5.0;
    rod.cross_section = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};

    PrismaticMeshParameters parameters;
    parameters.validity_max_depth = 5;
    parameters.rods.push_back(rod);
    const auto result = generate_prismatic_mesh(input, parameters);

    CHECK(result.report.rod_refined_segments > 0);
    CHECK(result.report.rod_refinement_max_depth > 0);
    CHECK(result.mesh.prisms.size() > 8);
}

TEST_CASE("multiway rod joint uses a shared tetrahedral transition", "[prismatic_mesh]")
{
    auto input = one_tet_input();
    input.edge_groups["rod"] = {{{0, 1}}, {{0, 2}}, {{0, 3}}};

    RodDefinition rod;
    rod.group = "rod";
    rod.thickness = 0.05;
    rod.cross_section = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};

    PrismaticMeshParameters parameters;
    parameters.rods.push_back(rod);
    const auto result = generate_prismatic_mesh(input, parameters);

    CHECK(result.report.rod_joint_tets == 12);
    CHECK(result.mesh.tets.size() == 12);
    CHECK(result.mesh.prisms.empty());
    size_t origin_vertices = 0;
    for (size_t v = 0; v < result.mesh.vertices.rows(); ++v) {
        origin_vertices += result.mesh.vertices.row(v).norm() < 1e-12;
    }
    CHECK(origin_vertices == 1);
}

TEST_CASE(
    "rod-shell junction is a conforming shared-offset transition",
    "[prismatic_mesh][rod][shell][junction]")
{
    auto input = one_tet_input();
    input.tet_groups = {{"ambient"}};
    input.face_groups["shell"] = {{{0, 1, 2}}};
    input.edge_groups["rod"] = {{{0, 3}}};

    RodDefinition rod;
    rod.group = "rod";
    rod.thickness = 0.1;
    rod.cross_section = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};

    PrismaticMeshParameters parameters;
    parameters.shells = {{"shell", 0.1}};
    parameters.rods = {rod};
    const auto result = generate_prismatic_mesh(input, parameters);

    CHECK(result.report.rod_shell_junctions == 1);
    CHECK(result.report.rod_shell_components == 1);
    CHECK(result.report.rod_shell_transition_tets > 0);
    CHECK(result.report.rod_shell_transition_tets == result.mesh.tets.size());
    CHECK(result.mesh.prisms.empty());
    CHECK(result.mesh.pyramids.empty());
    for (const auto& tet : result.mesh.tets) {
        CHECK(is_valid_tet(
            {{result.mesh.vertices.row(tet[0]),
              result.mesh.vertices.row(tet[1]),
              result.mesh.vertices.row(tet[2]),
              result.mesh.vertices.row(tet[3])}}));
    }
    for (const auto& [_, count] : facet_incidence(result.mesh)) CHECK(count <= 2);

    parameters.rods.front().thickness = 0.2;
    CHECK_THROWS(generate_prismatic_mesh(input, parameters));
}

TEST_CASE(
    "rod-shell transition and selected solid share the edited tet topology",
    "[prismatic_mesh][rod][shell][junction][solid]")
{
    PrismaticMeshInput input;
    input.vertices.resize(5, 3);
    input.vertices << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, -1;
    input.tets = {{{0, 1, 2, 3}}, {{0, 2, 1, 4}}};
    input.tet_groups = {{"ambient"}, {"solid"}};
    input.face_groups["shell"] = {{{0, 1, 2}}};
    input.edge_groups["rod"] = {{{0, 3}}};

    RodDefinition rod;
    rod.group = "rod";
    rod.thickness = 0.1;
    rod.cross_section = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};

    PrismaticMeshParameters parameters;
    parameters.solid_groups = {"solid"};
    parameters.shells = {{"shell", 0.1}};
    parameters.rods = {rod};
    const auto result = generate_prismatic_mesh(input, parameters);

    CHECK(result.report.rod_shell_transition_tets > 0);
    CHECK(result.report.solid_tets > 0);
    CHECK(
        result.mesh.tets.size() ==
        result.report.rod_shell_transition_tets + result.report.solid_tets);
    for (const auto& [_, count] : facet_incidence(result.mesh)) CHECK(count <= 2);
    CHECK(
        std::find(
            result.mesh.tet_region_tags.begin(),
            result.mesh.tet_region_tags.end(),
            "solid:solid") != result.mesh.tet_region_tags.end());
}

TEST_CASE("mixed msh output", "[prismatic_mesh]")
{
    HybridVolumeMesh mesh;
    mesh.vertices.resize(10, 3);
    mesh.vertices << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0.5,
        0.5, 2, 2, 0, 0;
    mesh.tets = {{{0, 1, 2, 3}}};
    mesh.prisms = {{{0, 1, 2, 3, 4, 5}}};
    mesh.pyramids = {{{0, 1, 6, 2, 8}}};
    mesh.tet_region_tags = {"solid"};
    mesh.prism_region_tags = {"shell"};
    mesh.pyramid_region_tags = {"transition"};

    const auto path = std::filesystem::temp_directory_path() / "wmtk_prismatic_mesh_mixed_test.msh";
    write_hybrid_msh(path, mesh);
    MshData data;
    data.load(path.string());
    CHECK(data.get_num_tets() == 1);
    CHECK(data.get_num_prisms() == 1);
    CHECK(data.get_num_pyramids() == 1);
    bool found_region_mapping = false;
    for (const auto& field : data.m_spec.element_data) {
        if (field.header.string_tags.empty() || field.header.string_tags.front() != "region_id") {
            continue;
        }
        REQUIRE(field.header.string_tags.size() == 2);
        const auto mapping = nlohmann::json::parse(field.header.string_tags[1]);
        CHECK(mapping.contains("solid"));
        CHECK(mapping.contains("shell"));
        CHECK(mapping.contains("transition"));
        found_region_mapping = true;
    }
    CHECK(found_region_mapping);
    std::filesystem::remove(path);
}
