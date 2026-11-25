#include <Eigen/Geometry>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <wmtk/components/prismatic_mesh/junction_protection.hpp>
#include <wmtk/components/prismatic_mesh/prism_jacobian.hpp>
#include <wmtk/components/prismatic_mesh/quality_improvement.hpp>
#include <wmtk/components/prismatic_mesh/rod_fairing.hpp>
#include <wmtk/components/prismatic_mesh/rod_joint_radius.hpp>
#include <wmtk/components/prismatic_mesh/rod_mesh.hpp>

using namespace wmtk::components::prismatic_mesh;
using wmtk::Vector3d;

namespace {
constexpr double pi = 3.14159265358979323846;
using Tet = std::array<size_t, 4>;
PrismaticMeshInput tube(size_t sections = 7, size_t degree = 6, bool mixed_split = false)
{
    PrismaticMeshInput input;
    input.vertices.resize(sections * (degree + 1), 3);
    input.vertex_tags.assign(input.vertices.rows(), 2);
    input.corr_input_vertex.assign(input.vertices.rows(), -1);
    input.corr_input_vid.assign(input.vertices.rows(), -1);
    input.input_to_offset_vertices.resize(input.vertices.rows());
    for (size_t i = 0; i < static_cast<size_t>(input.vertices.rows()); ++i)
        input.source_vertex_ids.push_back(100 + 7 * i); // Explicit IDs, not row indices.
    for (size_t i = 0; i < sections; ++i) {
        const size_t center = i * (degree + 1);
        input.vertices.row(center) << 0, 0, static_cast<double>(i);
        input.vertex_tags[center] = 1;
        input.input_vertices.push_back(center);
        for (size_t k = 0; k < degree; ++k) {
            const size_t v = center + k + 1;
            input.vertices.row(v) << .2 * std::cos(2 * pi * k / degree),
                .2 * std::sin(2 * pi * k / degree), static_cast<double>(i);
            input.offset_vertices.push_back(v);
            input.corr_input_vertex[v] = center;
            input.corr_input_vid[v] = input.source_vertex_ids[center];
            input.input_to_offset_vertices[center].push_back(v);
        }
    }
    std::vector<Tet> tets;
    for (size_t i = 1; i < sections; ++i) {
        const size_t u = (i - 1) * (degree + 1), v = i * (degree + 1);
        for (size_t k = 0; k < degree; ++k) {
            const size_t a = u + k + 1, b = u + (k + 1) % degree + 1;
            const size_t A = v + k + 1, B = v + (k + 1) % degree + 1;
            const std::array<size_t, 6> corners{u, a, b, v, A, B};
            std::array<size_t, 3> order{0, 1, 2};
            if (mixed_split) {
                const std::array<double, 3> rank{
                    degree / 2.0 - .5,
                    static_cast<double>(k),
                    static_cast<double>((k + 1) % degree)};
                std::sort(order.begin(), order.end(), [&](size_t x, size_t y) {
                    return rank[x] < rank[y];
                });
            }
            const auto p = order[0], q = order[1], r = order[2];
            std::vector<Tet> split{
                {corners[p], corners[q], corners[r], corners[r + 3]},
                {corners[p], corners[q], corners[r + 3], corners[q + 3]},
                {corners[p], corners[p + 3], corners[q + 3], corners[r + 3]}};
            int inversions = 0;
            for (int x = 0; x < 3; ++x)
                for (int y = x + 1; y < 3; ++y) inversions += order[x] > order[y];
            for (auto t : split) {
                if (inversions % 2) std::swap(t[0], t[1]);
                tets.push_back(t);
            }
        }
    }
    input.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i) {
        REQUIRE(tet_volume_above_threshold(input.vertices, tets[i], 0));
        for (int k = 0; k < 4; ++k) input.tetrahedra(i, k) = tets[i][k];
    }
    input.input_cells.assign(tets.size(), 0);
    input.offset_tet_tags.assign(tets.size(), 1);
    input.mesh = std::make_unique<wmtk::TetMesh>();
    input.mesh->init_with_isolated_vertices(input.vertices.rows(), tets);
    return input;
}
RodOptions options()
{
    RodOptions r;
    r.radius = .2;
    r.iterations = 2;
    return r;
}
void rebuild_fixture(PrismaticMeshInput& input, const std::vector<Tet>& tets)
{
    input.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i)
        for (int j = 0; j < 4; ++j) input.tetrahedra(i, j) = tets[i][j];
    input.mesh = std::make_unique<wmtk::TetMesh>();
    input.mesh->init_with_isolated_vertices(input.vertices.rows(), tets);
}
void reverse_fixture_vertex_order(PrismaticMeshInput& input)
{
    const size_t n = input.vertices.rows();
    const auto positions = input.vertices;
    for (size_t v = 0; v < n; ++v) input.vertices.row(n - 1 - v) = positions.row(v);
    std::reverse(input.vertex_tags.begin(), input.vertex_tags.end());
    std::reverse(input.source_vertex_ids.begin(), input.source_vertex_ids.end());
    std::reverse(input.corr_input_vid.begin(), input.corr_input_vid.end());
    std::reverse(input.corr_input_vertex.begin(), input.corr_input_vertex.end());
    input.input_vertices.clear();
    input.offset_vertices.clear();
    input.input_to_offset_vertices.assign(n, {});
    for (size_t v = 0; v < n; ++v) {
        auto& parent = input.corr_input_vertex[v];
        if (parent >= 0) parent = n - 1 - parent;
        if (input.vertex_tags[v] == 1) input.input_vertices.push_back(v);
        if (input.vertex_tags[v] == 2) {
            input.offset_vertices.push_back(v);
            input.input_to_offset_vertices[parent].push_back(v);
        }
    }
    std::vector<Tet> tets(input.tetrahedra.rows());
    for (size_t i = 0; i < tets.size(); ++i)
        for (int j = 0; j < 4; ++j) tets[i][j] = n - 1 - input.tetrahedra(i, j);
    rebuild_fixture(input, tets);
}
void refine_fixture_ring(PrismaticMeshInput& input, size_t a, size_t b)
{
    const size_t m = input.vertices.rows();
    input.vertices.conservativeResize(m + 1, 3);
    input.vertices.row(m) = (input.vertices.row(a) + input.vertices.row(b)) / 2;
    input.vertex_tags.push_back(2);
    input.source_vertex_ids.push_back(input.source_vertex_ids.back() + 7);
    input.corr_input_vid.push_back(input.corr_input_vid[a]);
    input.corr_input_vertex.push_back(input.corr_input_vertex[a]);
    input.input_to_offset_vertices.resize(m + 1);
    input.input_to_offset_vertices[input.corr_input_vertex[a]].push_back(m);
    input.offset_vertices.push_back(m);
    std::vector<Tet> tets;
    for (int i = 0; i < input.tetrahedra.rows(); ++i) {
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(i, j);
        if (std::find(t.begin(), t.end(), a) != t.end() &&
            std::find(t.begin(), t.end(), b) != t.end()) {
            Tet other = t;
            *std::find(t.begin(), t.end(), a) = m;
            *std::find(other.begin(), other.end(), b) = m;
            tets.push_back(other);
        }
        tets.push_back(t);
    }
    input.input_cells.assign(tets.size(), 0);
    input.offset_tet_tags.assign(tets.size(), 1);
    rebuild_fixture(input, tets);
}

void attach_shell_fixture(PrismaticMeshInput& input, size_t center, double side)
{
    const Vector3d origin = input.vertices.row(center);
    const auto vertex = [&](Vector3d delta, int tag) {
        const size_t v = input.vertices.rows();
        input.vertices.conservativeResize(v + 1, 3);
        input.vertices.row(v) = (origin + delta).transpose();
        input.vertex_tags.push_back(tag);
        input.source_vertex_ids.push_back(input.source_vertex_ids.back() + 7);
        input.corr_input_vertex.push_back(tag == 2 ? static_cast<int64_t>(center) : -1);
        input.corr_input_vid.push_back(tag == 2 ? input.source_vertex_ids[center] : -1);
        input.input_to_offset_vertices.resize(v + 1);
        if (tag == 1)
            input.input_vertices.push_back(v);
        else {
            input.offset_vertices.push_back(v);
            input.input_to_offset_vertices[center].push_back(v);
        }
        return v;
    };
    Tet attachment{
        center,
        vertex({1, -1, .3 * side}, 1),
        vertex({1, 1, .3 * side}, 1),
        vertex({.1, 0, .5 * side}, 2)};
    if (!tet_volume_above_threshold(input.vertices, attachment, 0))
        std::swap(attachment[0], attachment[1]);
    REQUIRE(tet_volume_above_threshold(input.vertices, attachment, 0));
    std::vector<Tet> cells(input.tetrahedra.rows());
    for (size_t i = 0; i < cells.size(); ++i)
        for (int j = 0; j < 4; ++j) cells[i][j] = input.tetrahedra(i, j);
    cells.push_back(attachment);
    input.input_cells.push_back(0);
    input.offset_tet_tags.push_back(1);
    rebuild_fixture(input, cells);
}


} // namespace

TEST_CASE(
    "Rod layers use the transported ring slots independently of old interior splits",
    "[rod][rings]")
{
    auto input = tube(7, 6, true);
    SECTION("original vertex rows") {}
    SECTION("reversed vertex rows")
    {
        reverse_fixture_vertex_order(input);
    }
    const auto positions = input.vertices;
    const auto ids = input.source_vertex_ids;
    preprocess_rod_mesh(input, options());
    construct_rod_frame_topology(input);
    auto hybrid = build_prism_dominant_mesh(input, 0);
    CHECK(input.source_vertex_ids == ids);
    CHECK(input.input_vertices.size() == 7);
    CHECK((input.vertices - positions).norm() < 1e-12);
    REQUIRE(input.rod_state->frames.size() == 7);
    for (const auto& frame : input.rod_state->frames) {
        CHECK(frame.valid);
        CHECK(frame.ring.size() == 6);
    }
    CHECK(hybrid.report["accepted_layers"] == 6);
    CHECK(hybrid.report["prisms"] == 36);
    CHECK(hybrid.report["band_tetrahedra"] == 0);
    for (const auto& cell : hybrid.cells) {
        CHECK(cell.type == HybridCellType::Prism);
        CHECK(minimum_prism_jacobian(input.vertices, cell).determinant > 0);
    }
    validate_prism_dominant_mesh(input, hybrid, 0);
}

TEST_CASE(
    "A rod layer rolls back as a whole when no positive reference split meets the floor",
    "[rod][rings]")
{
    auto input = tube(3);
    auto opt = options();
    opt.iterations = 0;
    preprocess_rod_mesh(input, opt);
    const auto positions = input.vertices;
    const auto cells = input.tetrahedra;
    auto hybrid = build_prism_dominant_mesh(input, 1);
    CHECK(hybrid.report["accepted_layers"] == 0);
    CHECK(hybrid.report["prisms"] == 0);
    CHECK(hybrid.cells.size() == size_t(cells.rows()));
    CHECK(input.vertices == positions);
    CHECK(input.tetrahedra == cells);
    for (const auto& cell : hybrid.cells) CHECK(cell.type == HybridCellType::Tetrahedron);
    validate_prism_dominant_mesh(input, hybrid, 1);
}

TEST_CASE("An original-correspondence triangle patch collapses to one vertex ring", "[rod][rings]")
{
    auto input = tube(3);
    const std::array<size_t, 3> face = {8, 9, 16};
    const size_t point = input.vertices.rows();
    input.vertices.conservativeResize(point + 1, 3);
    input.vertices.row(point) =
        (input.vertices.row(face[0]) + input.vertices.row(face[1]) + input.vertices.row(face[2])) /
        3;
    input.vertex_tags.push_back(2);
    input.source_vertex_ids.push_back(input.source_vertex_ids.back() + 7);
    input.corr_input_vertex.push_back(7);
    input.corr_input_vid.push_back(input.source_vertex_ids[7]);
    input.offset_vertices.push_back(point);
    input.input_to_offset_vertices.resize(point + 1);
    input.input_to_offset_vertices[7].push_back(point);
    std::vector<Tet> cells;
    for (int i = 0; i < input.tetrahedra.rows(); ++i) {
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(i, j);
        if (std::all_of(face.begin(), face.end(), [&](size_t v) {
                return std::find(t.begin(), t.end(), v) != t.end();
            })) {
            for (size_t corner : face) {
                Tet child = t;
                *std::find(child.begin(), child.end(), corner) = point;
                REQUIRE(tet_volume_above_threshold(input.vertices, child, 0));
                cells.push_back(child);
            }
        } else
            cells.push_back(t);
    }
    input.input_cells.assign(cells.size(), 0);
    input.offset_tet_tags.assign(cells.size(), 1);
    rebuild_fixture(input, cells);
    const auto centers = input.input_vertices;
    const auto positions = input.vertices;
    const auto parents = input.corr_input_vid;
    preprocess_rod_mesh(input, options());
    const auto& state = *input.rod_state;
    CHECK(state.report["vertex_regions"]["passes"][0]["collapses"] == 1);
    REQUIRE(state.vertex_frame.count(7) == 1);
    CHECK(state.frames[state.vertex_frame.at(7)].ring.size() == 6);
    CHECK(input.corr_input_vid == parents);
    for (size_t v : centers) CHECK(input.vertices.row(v) == positions.row(v));
}

TEST_CASE(
    "Ring degree alignment adds offset points without inserting centerline sections",
    "[rod][rings]")
{
    auto input = tube(3, 4);
    refine_fixture_ring(input, 6, 7);
    const auto centers = input.input_vertices;
    const auto original_positions = input.vertices;
    preprocess_rod_mesh(input, options());
    CHECK(input.input_vertices == centers);
    REQUIRE(input.rod_state->frames.size() == 3);
    for (const auto& frame : input.rod_state->frames) {
        REQUIRE(frame.valid);
        CHECK(frame.ring.size() == 5);
        for (size_t v : frame.ring) {
            CHECK(input.vertex_tags[v] == 2);
            CHECK(input.corr_input_vertex[v] == int64_t(frame.center));
            CHECK(input.corr_input_vid[v] == input.source_vertex_ids[frame.center]);
        }
    }
    CHECK(input.rod_state->report["ring_edge_splits"] == 2);
    for (size_t v : centers) CHECK(input.vertices.row(v) == original_positions.row(v));
}

namespace {
PrismaticMeshInput swapped_interface_fixture()
{
    auto input = tube(2);
    // A small twist keeps both triangulations of this interface geometrically valid.
    for (size_t k = 0; k < 6; ++k) {
        input.vertices(8 + k, 0) = .2 * std::cos(2 * pi * k / 6 - .2);
        input.vertices(8 + k, 1) = .2 * std::sin(2 * pi * k / 6 - .2);
    }
    constexpr size_t A = 1, B = 2, D = 9, C = 10;
    const size_t apex = input.vertices.rows();
    input.vertices.conservativeResize(apex + 1, 3);
    input.vertices.row(apex) << .6 * std::cos(pi / 3 - .1), .6 * std::sin(pi / 3 - .1), .5;
    input.vertex_tags.push_back(-1);
    input.source_vertex_ids.push_back(input.source_vertex_ids.back() + 7);
    input.corr_input_vid.push_back(-1);
    input.corr_input_vertex.push_back(-1);
    input.input_to_offset_vertices.resize(apex + 1);
    std::vector<Tet> cells;
    for (int i = 0; i < input.tetrahedra.rows(); ++i) {
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(i, j);
        // Replace the two band tets touching the desired B--D edge by the
        // alternate A--C triangulation, as in the observed surface mismatch.
        if (std::find(t.begin(), t.end(), B) != t.end() &&
            std::find(t.begin(), t.end(), D) != t.end())
            continue;
        cells.push_back(t);
    }
    auto add = [&](Tet t) {
        if (!tet_volume_above_threshold(input.vertices, t, 0)) std::swap(t[0], t[1]);
        REQUIRE(tet_volume_above_threshold(input.vertices, t, 0));
        cells.push_back(t);
    };
    add({0, A, C, B});
    add({0, A, D, C});
    const size_t band_count = cells.size();
    add({apex, A, B, C});
    add({apex, A, C, D});
    input.input_cells.assign(cells.size(), 0);
    input.offset_tet_tags.assign(cells.size(), 1);
    for (size_t i = band_count; i < cells.size(); ++i) input.offset_tet_tags[i] = -1;
    rebuild_fixture(input, cells);
    return input;
}
} // namespace

TEST_CASE("Interface swaps update both volume stars at fixed positions", "[rod][rings][swap]")
{
    auto input = swapped_interface_fixture();
    auto opt = options();
    opt.iterations = 0;
    preprocess_rod_mesh(input, opt);
    const auto positions = input.vertices;
    const auto cells = input.tetrahedra;
    const auto correspondence = input.corr_input_vid;
    double floor = 0;
    SECTION("a positive 4-4 swap restores the ring correspondence") {}
    SECTION("a rejected volume floor leaves the complete layer unchanged")
    {
        floor = 1;
    }
    const auto report = align_rod_interfaces_by_swaps(input, floor);
    CHECK(input.vertices == positions);
    CHECK(input.corr_input_vid == correspondence);
    if (floor > 0) {
        CHECK(report["swaps"] == 0);
        CHECK(report["failed_layers"] == 1);
        CHECK(input.tetrahedra == cells);
    } else {
        CHECK(report["swaps"] == 1);
        CHECK(report["repaired_layers"] == 1);
        CHECK(report["failed_layers"] == 0);
        const auto& op = report["operations"][0];
        CHECK(op["old_band_tets"] == 2);
        CHECK(op["new_band_tets"] == 2);
        CHECK(op["old_background_tets"] == 2);
        CHECK(op["new_background_tets"] == 2);
        CHECK(input.tetrahedra.rows() == cells.rows());
    }
    CHECK(input.mesh->check_mesh_connectivity_validity());
    for (int i = 0; i < input.tetrahedra.rows(); ++i) {
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(i, j);
        CHECK(tet_volume_above_threshold(input.vertices, t, 0));
    }
}

TEST_CASE("Protected rod attachments retain their original offset vertices", "[rod][rings][mixed]")
{
    auto input = tube();
    attach_shell_fixture(input, 0, -1);
    const auto positions = input.vertices;
    preprocess_rod_mesh(input, options());
    REQUIRE(input.rod_state->interface_vertices.count(0) == 1);
    REQUIRE_FALSE(input.rod_state->frozen_offset.empty());
    construct_rod_frame_topology(input);
    verify_junction_one_rings(input);
    for (size_t v : input.rod_state->frozen_offset)
        CHECK(input.vertices.row(v) == positions.row(v));
}

TEST_CASE("Free rod disk regions keep their rim and end at prism faces", "[rod][rings][ends]")
{
    constexpr size_t sections = 4, degree = 6, stride = degree + 1;
    auto input = tube(sections, degree);
    std::vector<Tet> cells(input.tetrahedra.rows());
    for (size_t i = 0; i < cells.size(); ++i)
        for (int j = 0; j < 4; ++j) cells[i][j] = input.tetrahedra(i, j);
    for (size_t center : {size_t(0), (sections - 1) * stride}) {
        const size_t apex = input.vertices.rows();
        input.vertices.conservativeResize(apex + 1, 3);
        input.vertices.row(apex) = input.vertices.row(center);
        input.vertices(apex, 2) += center == 0 ? -.3 : .3;
        input.vertex_tags.push_back(2);
        input.source_vertex_ids.push_back(input.source_vertex_ids.back() + 7);
        input.corr_input_vertex.push_back(center);
        input.corr_input_vid.push_back(input.source_vertex_ids[center]);
        input.input_to_offset_vertices.resize(apex + 1);
        input.input_to_offset_vertices[center].push_back(apex);
        input.offset_vertices.push_back(apex);
        for (size_t k = 0; k < degree; ++k) {
            Tet tet{center, center + k + 1, center + (k + 1) % degree + 1, apex};
            if (!tet_volume_above_threshold(input.vertices, tet, 0)) std::swap(tet[0], tet[1]);
            cells.push_back(tet);
        }
    }
    input.input_cells.assign(cells.size(), 0);
    input.offset_tet_tags.assign(cells.size(), 1);
    rebuild_fixture(input, cells);
    auto opt = options();
    opt.iterations = 0;
    preprocess_rod_mesh(input, opt);
    REQUIRE(input.rod_state->vertex_frame.size() == sections);
    auto hybrid = build_prism_dominant_mesh(input, 0);
    REQUIRE(hybrid.report["prisms"] == (sections - 1) * degree);
    REQUIRE(hybrid.report["band_tetrahedra"] == 2 * degree);
    const auto positions = input.vertices.eval();
    const auto topology = input.tetrahedra.eval();
    size_t completed = 2;
    SECTION("both ends have complete native prism fans") {}
    SECTION("a mismatched end retains its cap")
    {
        auto& state = *input.rod_state;
        auto& ring = state.frames.at(state.vertex_frame.at(0)).ring;
        std::swap(ring[1], ring[2]);
        completed = 1;
    }
    finalize_rod_ends(input, hybrid);
    CHECK(input.vertices == positions);
    CHECK(input.tetrahedra == topology);
    CHECK(input.rod_state->report["completed_prismatic_free_ends"] == completed);
    CHECK(hybrid.report["band_tetrahedra"] == (2 - completed) * degree);
    CHECK(hybrid.report["background_tetrahedra"] == completed * degree);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
    finalize_rod_ends(input, hybrid);
    CHECK(input.rod_state->report["completed_prismatic_free_ends"] == completed);
}

TEST_CASE("Shell-owned offsets preserve incident rod sections", "[rod][rings][mixed]")
{
    auto input = tube(3);
    input.rod_state = std::make_shared<RodMeshState>();
    auto& state = *input.rod_state;
    state.input_edges = {{0, 7}};
    state.input_vertices = {0, 7};
    // These offset vertices belong to the neighboring shell region, but their
    // common edge is also incident to a tet containing the rod axis {0, 7}.
    for (size_t v : {8, 9}) {
        input.corr_input_vertex[v] = 14;
        input.corr_input_vid[v] = input.source_vertex_ids[14];
    }
    CHECK_FALSE(is_rod_offset_vertex(input, 8));
    CHECK_FALSE(is_rod_offset_vertex(input, 9));
    CHECK_FALSE(rod_collapse_allowed(input, 8, 9));
    CHECK_FALSE(rod_collapse_allowed(input, 9, 8));
    CHECK(rod_collapse_allowed(input, 15, 16));
    state.frozen_offset.insert(15);
    CHECK_FALSE(rod_collapse_allowed(input, 15, 16));
    CHECK_FALSE(rod_collapse_allowed(input, 16, 15));
}

TEST_CASE(
    "Atomic rod layers compose with shell prisms and unchanged input volume",
    "[rod][rings][mixed]")
{
    auto input = tube(3);
    std::vector<Tet> cells(input.tetrahedra.rows());
    for (int i = 0; i < input.tetrahedra.rows(); ++i)
        for (int j = 0; j < 4; ++j) cells[i][j] = input.tetrahedra(i, j);
    auto vertex = [&](const Vector3d& p, int tag, int64_t parent = -1) {
        const size_t v = input.vertices.rows();
        input.vertices.conservativeResize(v + 1, 3);
        input.vertices.row(v) = p.transpose();
        input.vertex_tags.push_back(tag);
        input.source_vertex_ids.push_back(input.source_vertex_ids.back() + 7);
        input.corr_input_vertex.push_back(parent);
        input.corr_input_vid.push_back(parent < 0 ? -1 : input.source_vertex_ids[parent]);
        input.input_to_offset_vertices.resize(v + 1);
        if (tag == 1)
            input.input_vertices.push_back(v);
        else {
            input.offset_vertices.push_back(v);
            input.input_to_offset_vertices[parent].push_back(v);
        }
        return v;
    };
    const size_t a = vertex({3, 0, 0}, 1), b = vertex({4, 0, 0}, 1), c = vertex({3, 1, 0}, 1);
    const size_t A = vertex({3, 0, .2}, 2, a), B = vertex({4, 0, .2}, 2, b),
                 C = vertex({3, 1, .2}, 2, c);
    cells.push_back({a, b, c, C});
    cells.push_back({a, b, C, B});
    cells.push_back({a, A, B, C});
    const Tet volume = {
        vertex({6, 0, 0}, 1),
        vertex({7, 0, 0}, 1),
        vertex({6, 1, 0}, 1),
        vertex({6, 0, 1}, 1)};
    cells.push_back(volume);
    input.input_cells.assign(cells.size(), 0);
    input.input_cells.back() = 1;
    input.offset_tet_tags.assign(cells.size(), 1);
    input.offset_tet_tags.back() = -1;
    rebuild_fixture(input, cells);
    const auto original = input.vertices;
    auto opt = options();
    opt.iterations = 0;
    bool staged_quality = false;
    SECTION("original correspondence rods") {}
    SECTION("edge-based rods follow shell construction and remeshing")
    {
        opt.edge_based = true;
        staged_quality = true;
        input.source_faces = {{a, b, c}};
        input.source_edges = {{0, 7}, {7, 14}, {a, b}, {b, c}, {a, c}};
    }
    preprocess_rod_mesh(input, opt);
    PrismDominantMesh hybrid;
    if (staged_quality) {
        REQUIRE(input.rod_state->input_edges.size() == 4);
        REQUIRE(input.source_faces == std::vector<std::array<size_t, 3>>{{a, b, c}});
        const auto before_shell = input.vertices;
        evaluate_target_positions(input, .1);
        OptimizationOptions shell_construction;
        shell_construction.iterations = 1;
        optimize_prismatic_mesh(input, shell_construction);
        REQUIRE(input.rod_state->report["shared_construction_rod_geometry_deferred"] == true);
        for (size_t v : input.offset_vertices)
            if (is_rod_offset_vertex(input, v))
                REQUIRE(input.vertices.row(v) == before_shell.row(v));
        hybrid = build_shell_prism_mesh(input, 0);
        REQUIRE(std::count_if(hybrid.cells.begin(), hybrid.cells.end(), [](const auto& cell) {
                    return cell.type == HybridCellType::Prism;
                }) == 1);
        input.target_thickness = .2;
        label_offset_faces(input);
        QualityImprovementOptions quality;
        quality.cycles = 1;
        quality.mixed_smoothing_sweeps = 2;
        quality.surface_split_budget = 10;
        quality.column_collapse_attempts = quality.surface_flip_attempts = 10;
        quality.volume_split_budget = 10;
        quality.volume_passes = 1;
        quality.time_limit = 10;
        const auto improved = improve_prismatic_mesh_quality(input, hybrid, quality);
        REQUIRE(improved.report["reserved_rod_cells"].get<size_t>() > 0);
        REQUIRE(improved.report["stages"].size() == 4);
        hybrid = adopt_improved_shell_mesh(input, improved);
        reserve_shell_cells(input, hybrid);
        const auto shell_positions = input.vertices;
        optimize_rod_tet_positions(input);
        for (size_t v : input.rod_state->fixed_shell_vertices)
            CHECK(input.vertices.row(v) == shell_positions.row(v));
        hybrid = build_rod_prism_mesh(input, 0);
        CHECK(
            hybrid.report["checks"]["completed_shell_cells_preserved_during_rod_reconstruction"] ==
            true);
        const auto output = finalize_staged_mixed_mesh(input, hybrid, &improved);
        CHECK(output.cells.size() == hybrid.cells.size());
        CHECK(output.report["shell_stage"]["reserved_rod_cells"].get<size_t>() > 0);
        CHECK(hybrid.report["prisms"].get<size_t>() > 1);
    } else {
        hybrid = build_prism_dominant_mesh(input, 0);
        CHECK(hybrid.report["prisms"] == 13);
    }
    CHECK(hybrid.report["input_tetrahedra"] == 1);
    if (!staged_quality) {
        CHECK(hybrid.report["band_tetrahedra"] == 0);
        CHECK(input.vertices == original);
    }
    bool found_volume = false;
    std::set<int64_t> candidate_ids;
    for (const auto& cell : hybrid.cells) {
        if (cell.type == HybridCellType::Prism)
            CHECK(candidate_ids.insert(cell.prism_candidate).second);
        if (cell.type == HybridCellType::Tetrahedron &&
            cell.vertices == std::vector<size_t>(volume.begin(), volume.end()))
            found_volume = true;
    }
    CHECK(found_volume);
    validate_prism_dominant_mesh(input, hybrid, 0);
}

TEST_CASE("Edge-based rod attachment preserves shell-owned offsets", "[rod][edge_based][mixed]")
{
    auto input = tube(4);
    attach_shell_fixture(input, 7, -1);
    const size_t last = input.tetrahedra.rows() - 1;
    Tet shell;
    for (int j = 0; j < 4; ++j) shell[j] = input.tetrahedra(last, j);
    std::vector<size_t> face;
    for (size_t& v : shell) {
        if (input.vertex_tags[v] == 2)
            v = 8; // A real rod-link point is also shell-owned.
        else
            face.push_back(v);
    }
    if (!tet_volume_above_threshold(input.vertices, shell, 0)) std::swap(shell[0], shell[1]);
    REQUIRE(tet_volume_above_threshold(input.vertices, shell, 0));
    std::vector<Tet> cells(input.tetrahedra.rows());
    for (size_t tid = 0; tid < cells.size(); ++tid)
        for (int j = 0; j < 4; ++j) cells[tid][j] = input.tetrahedra(tid, j);
    cells.back() = shell;
    rebuild_fixture(input, cells);
    input.source_faces = {{face[0], face[1], face[2]}};
    input.source_edges =
        {{0, 7}, {7, 14}, {14, 21}, {face[0], face[1]}, {face[1], face[2]}, {face[2], face[0]}};
    const auto positions = input.vertices;
    auto opt = options();
    opt.edge_based = true;
    opt.iterations = 0;
    preprocess_rod_mesh(input, opt);
    REQUIRE(input.rod_state->input_edges.size() == 6);
    REQUIRE(input.rod_state->interface_vertices.count(7) == 1);
    REQUIRE(
        input.rod_state->report["edge_based_preparation"]["shell_port_separation_splits"]
            .get<size_t>() > 0);
    REQUIRE(input.corr_input_vertex[8] == 7);
    REQUIRE(input.vertices.topRows(positions.rows()) == positions);
    std::sort(shell.begin(), shell.end());
    bool retained = false;
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(tid, j);
        std::sort(t.begin(), t.end());
        retained |= t == shell;
    }
    REQUIRE(retained);
    REQUIRE(input.mesh->check_mesh_connectivity_validity());
}

TEST_CASE("Edge-based rod preparation preserves closed disjoint tet links", "[rod][edge_based]")
{
    auto input = tube(4, 5, true);
    SECTION("separate frames sharing the complete middle ring")
    {
        input = tube(3);
        std::vector<Tet> cells;
        for (int i = 0; i < 18; ++i) {
            Tet t;
            for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(i, j);
            cells.push_back(t);
            // Reflect the first segment at its upper section. Both source
            // edges initially have precisely the same six link vertices.
            for (size_t& v : t)
                if (v < 7) v += 14;
            std::swap(t[0], t[1]);
            REQUIRE(tet_volume_above_threshold(input.vertices, t, 0));
            cells.push_back(t);
        }
        rebuild_fixture(input, cells);
    }
    SECTION("already disjoint frame links") {}
    const size_t original_sections = input.input_vertices.size();
    const auto original = input.vertices;
    const auto volume = [](const PrismaticMeshInput& mesh) {
        double sum = 0;
        for (Eigen::Index i = 0; i < mesh.tetrahedra.rows(); ++i) {
            const Vector3d a =
                mesh.vertices.row(mesh.tetrahedra(i, 1)) - mesh.vertices.row(mesh.tetrahedra(i, 0));
            const Vector3d b =
                mesh.vertices.row(mesh.tetrahedra(i, 2)) - mesh.vertices.row(mesh.tetrahedra(i, 0));
            const Vector3d c =
                mesh.vertices.row(mesh.tetrahedra(i, 3)) - mesh.vertices.row(mesh.tetrahedra(i, 0));
            sum += a.dot(b.cross(c)) / 6;
        }
        return sum;
    };
    const double before = volume(input);
    auto opt = options();
    opt.edge_based = true;
    opt.iterations = 0;
    preprocess_rod_mesh(input, opt);
    REQUIRE(input.rod_state->section_edges.size() == original_sections - 1);
    CHECK((input.vertices.topRows(original.rows()).array() == original.array()).all());
    CHECK(std::abs(volume(input) - before) < 1e-12);
    CHECK(
        input.rod_state->report["edge_based_preparation"]["shared_frame_vertex_incidences_after"] ==
        0);
    std::set<size_t> used;
    for (const auto& [center, edge] : input.rod_state->section_edges) {
        REQUIRE(input.rod_state->vertex_frame.count(center));
        const auto& frame = input.rod_state->frames[input.rod_state->vertex_frame.at(center)];
        REQUIRE(frame.ring.size() >= 3);
        for (size_t v : frame.ring) {
            CHECK(used.insert(v).second);
            CHECK(input.corr_input_vertex[v] == int64_t(center));
        }
        for (size_t k = 0; k < frame.ring.size(); ++k) {
            Tet tet{edge[0], edge[1], frame.ring[k], frame.ring[(k + 1) % frame.ring.size()]};
            std::sort(tet.begin(), tet.end());
            size_t count = 0;
            for (Eigen::Index i = 0; i < input.tetrahedra.rows(); ++i) {
                Tet actual;
                for (int j = 0; j < 4; ++j) actual[j] = input.tetrahedra(i, j);
                std::sort(actual.begin(), actual.end());
                count += actual == tet;
            }
            CHECK(count == 1);
        }
    }
    for (Eigen::Index i = 0; i < input.tetrahedra.rows(); ++i) {
        Tet tet;
        for (int j = 0; j < 4; ++j) tet[j] = input.tetrahedra(i, j);
        CHECK(tet_volume_above_threshold(input.vertices, tet, 0));
    }
    verify_junction_one_rings(input);
}

TEST_CASE(
    "Edge-based rod conversion validates native cells and retains failed layers",
    "[rod][edge_based]")
{
    auto input = swapped_interface_fixture();
    auto opt = options();
    opt.edge_based = true;
    double floor = 0;
    SECTION("native conversion with background") {}
    SECTION("infeasible layer volume floor")
    {
        floor = 1;
    }
    preprocess_rod_mesh(input, opt);
    construct_rod_frame_topology(input);
    const auto positions = input.vertices;
    const auto reference = input.tetrahedra;
    auto hybrid = build_prism_dominant_mesh(input, floor);
    validate_prism_dominant_mesh(input, hybrid, floor);
    CHECK((input.vertices.topRows(positions.rows()).array() == positions.array()).all());
    if (floor > 0) {
        CHECK(hybrid.report["accepted_layers"] == 0);
        CHECK(input.tetrahedra == reference);
        for (const auto& cell : hybrid.cells) CHECK(cell.type == HybridCellType::Tetrahedron);
    }
    for (const auto& cell : hybrid.cells)
        if (cell.type == HybridCellType::Prism)
            CHECK(minimum_prism_jacobian(input.vertices, cell).determinant > 0);
}

TEST_CASE("Rod region preparation rejects invalid radius and correspondence", "[rod][rings]")
{
    auto input = tube();
    auto opt = options();
    SECTION("negative radius")
    {
        opt.radius = -.1;
    }
    SECTION("nonfinite radius")
    {
        opt.radius = std::numeric_limits<double>::quiet_NaN();
    }
    SECTION("negative position iterations")
    {
        opt.position_iterations = -1;
    }
    SECTION("negative neighborhood radius")
    {
        opt.background_remeshing.radius = -1;
    }
    SECTION("nonfinite neighborhood length")
    {
        opt.background_remeshing.target_length = std::numeric_limits<double>::quiet_NaN();
    }
    SECTION("zero neighborhood time limit")
    {
        opt.background_remeshing.time_limit = 0;
    }
    SECTION("negative position weight")
    {
        opt.position_weight = -1;
    }
    SECTION("negative smoothing weight")
    {
        opt.position_smoothing_weight = -1;
    }
    SECTION("negative smoothing iterations")
    {
        opt.position_smoothing_iterations = -1;
    }
    SECTION("negative smoothing geometry weight")
    {
        opt.position_smoothing_geometry_weight = -1;
    }
    SECTION("wrong parent source ID")
    {
        input.corr_input_vid[1] = -1;
    }
    CHECK_THROWS(preprocess_rod_mesh(input, opt));
}

TEST_CASE(
    "Rod tet positions improve after degree alignment without changing mapping",
    "[rod][edge_based][positions]")
{
    auto input = tube(4, 6, true);
    auto opt = options();
    opt.edge_based = true;
    opt.position_iterations = 8;
    SECTION("position targets alone") {}
    SECTION("with longitudinal and circular fairing")
    {
        opt.position_smoothing_weight = 100;
    }
    preprocess_rod_mesh(input, opt);
    construct_rod_frame_topology(input);
    finalize_rod_frame_mapping(input);
    // A transition ring also becomes a prism cap. Its last radial edge must
    // follow the transported slot just like edge zero on a source section.
    const auto transition = std::find_if(
        input.rod_state->frames.begin(),
        input.rod_state->frames.end(),
        [&](const auto& f) {
            return f.valid && !input.rod_state->mapping_controls.count(f.center);
        });
    REQUIRE(transition != input.rod_state->frames.end());
    const size_t v = transition->ring.back();
    input.vertices(v, 0) += .005;
    input.vertices(v, 2) += .01;
    const auto positions = input.vertices;
    const auto tets = input.tetrahedra;
    std::vector<std::vector<size_t>> mapping;
    for (const auto& f : input.rod_state->frames) mapping.push_back(f.ring);
    optimize_rod_tet_positions(input);
    const auto& report = input.rod_state->report
                             [opt.position_smoothing_weight > 0 ? "tet_surface_smoothing"
                                                                : "tet_position_optimization"];
    REQUIRE(report["accepted_moves"].get<size_t>() > 0);
    CHECK(report["candidate_geometry_before"]["unequal_degree_layers"] == 0);
    CHECK(report["energy_after"][0].get<double>() < report["energy_before"][0].get<double>());
    CHECK(report["intermediate_laplace_vertices"].get<size_t>() > 0);
    double previous = report["energy_before"][0];
    for (const auto& row : report["history"]) {
        const double energy = row["total_energy"];
        CHECK(energy <= previous + 1e-9);
        previous = energy;
    }
    CHECK(input.tetrahedra == tets);
    for (size_t i = 0; i < mapping.size(); ++i)
        CHECK(input.rod_state->frames[i].ring == mapping[i]);
    CHECK(report["transition_frame_target_vertices"].get<size_t>() > 0);
    for (const auto& f : input.rod_state->frames) {
        if (!f.valid) continue;
        for (size_t k = 0; k < f.ring.size(); ++k) {
            const double angle = 2 * pi * k / f.ring.size();
            const Vector3d expected =
                Vector3d(input.vertices.row(f.center)) +
                input.rod_state->radius *
                    (f.rotation * Vector3d(std::cos(angle), std::sin(angle), 0));
            CHECK((input.rod_state->frame_targets.at(f.ring[k]) - expected).norm() < 1e-12);
        }
    }
    for (size_t u : input.input_vertices) CHECK(input.vertices.row(u) == positions.row(u));
    for (Eigen::Index i = 0; i < tets.rows(); ++i) {
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = tets(i, j);
        CHECK(tet_volume_above_threshold(input.vertices, t, 0));
    }
}

TEST_CASE("Rod fairing follows real bends while detecting surface ripples", "[rod][fairing]")
{
    PrismaticMeshInput input;
    input.rod_state = std::make_shared<RodMeshState>();
    auto& state = *input.rod_state;
    state.radius = .2;
    input.vertices.resize(21, 3);
    bool bend = false;
    SECTION("nonuniform straight sampling") {}
    SECTION("bent centerline")
    {
        bend = true;
    }
    for (size_t i = 0; i < 3; ++i) {
        const size_t center = 7 * i;
        const Vector3d origin = bend && i == 2 ? Vector3d(1, 0, 1) : Vector3d(0, 0, i == 2 ? 3 : i);
        const Vector3d axis =
            bend ? Vector3d(double(i), 0, double(2 - i)).normalized() : Vector3d::UnitZ();
        input.vertices.row(center) = origin.transpose();
        RodFrame frame;
        frame.center = center;
        frame.valid = true;
        frame.rotation = Eigen::Quaterniond::FromTwoVectors(Vector3d::UnitZ(), axis);
        for (size_t k = 0; k < 6; ++k) {
            const double angle = 2 * pi * k / 6;
            const size_t v = center + k + 1;
            input.vertices.row(v) =
                (origin +
                 state.radius * (frame.rotation * Vector3d(std::cos(angle), std::sin(angle), 0)))
                    .transpose();
            frame.ring.push_back(v);
        }
        state.vertex_frame[center] = state.frames.size();
        state.frames.push_back(frame);
    }
    state.directed_input_edges = {{0, 7}, {7, 14}};
    state.rods = {{0, 1}};
    const auto score = [&]() {
        double result = 0;
        for (const auto& row : rod_fairing_rows(input))
            result += row.weight * row.residual(input.vertices).squaredNorm();
        return result;
    };
    CHECK(score() < 1e-24);
    input.vertices(8, 0) += .02;
    const double rippled = score();
    CHECK(rippled > 1e-4);
    input.vertices *= 3.3;
    input.vertices.rowwise() += Vector3d(7, -3, 2).transpose();
    state.radius *= 3.3;
    CHECK(std::abs(score() - rippled) < 1e-12);
}

TEST_CASE(
    "Joint geometry can improve while its star remains protected from topology changes",
    "[rod][joint_smoothing]")
{
    auto input = tube(5, 6);
    auto opt = options();
    opt.edge_based = true;
    opt.joint_smoothing_iterations = 8;
    preprocess_rod_mesh(input, opt);
    construct_rod_frame_topology(input);
    finalize_rod_frame_mapping(input);
    // A protected interior section supplies a small joint-star geometry fixture;
    // the actual branching graph is covered by the FlowLoops integration run.
    const size_t center = 14;
    auto& state = *input.rod_state;
    state.joints.insert(center);
    auto& protection = *input.junction_protection;
    protection.centers.insert(center);
    for (int i = 0; i < input.tetrahedra.rows(); ++i) {
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(i, j);
        if (std::find(t.begin(), t.end(), center) == t.end()) continue;
        std::sort(t.begin(), t.end());
        protection.original_band_cells.push_back(t);
        for (size_t v : t)
            if (input.vertex_tags[v] == 2) {
                protection.offset_vertices.insert(v);
                state.frozen_offset.insert(v);
                protection.offset_positions[v] = input.vertices.row(v);
            }
    }
    REQUIRE_FALSE(protection.offset_vertices.empty());
    // The accumulated reference can be farther than the old 0.75r cap. It must
    // not block an otherwise valid descent move in this later geometry solve.
    for (size_t v : protection.offset_vertices)
        state.joint_position_reference[v] =
            Vector3d(input.vertices.row(v)) + Vector3d(4 * state.radius, 0, 0);
    const auto before = input.vertices;
    const auto cells = input.tetrahedra;
    optimize_rod_joint_positions(input);
    const auto& report = state.report["joint_smoothing"];
    CHECK(report["protected_offset_vertices_moved"].get<size_t>() > 0);
    CHECK(report["maximum_joint_displacement_limit"].is_null());
    CHECK(report["maximum_total_joint_displacement"].get<double>() > .75 * state.radius);
    CHECK(report["energy_after"][0].get<double>() < report["energy_before"][0].get<double>());
    CHECK(input.tetrahedra == cells);
    for (size_t v = 0; v < size_t(input.vertices.rows()); ++v)
        if (input.vertex_tags[v] == 1 || !protection.offset_vertices.count(v))
            CHECK(input.vertices.row(v) == before.row(v));
    for (size_t v : protection.offset_vertices) CHECK_FALSE(junction_collapse_allowed(input, v));
    CHECK_NOTHROW(verify_junction_one_rings(input));
    const size_t v = *protection.offset_vertices.begin();
    const auto accepted = input.vertices;
    input.vertices(v, 0) += 1e-5;
    CHECK_THROWS(verify_junction_one_rings(input));
    input.vertices = accepted;
    CHECK_NOTHROW(verify_junction_one_rings(input));
    CHECK(junction_protection_report(input)["offset_positions_fixed"] == false);
    for (int i = 0; i < cells.rows(); ++i) {
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = cells(i, j);
        CHECK(tet_volume_above_threshold(input.vertices, t, 0));
    }
}

TEST_CASE(
    "Joint tube-union gradients follow the actual capsule distances",
    "[rod][joint_smoothing]")
{
    const double radius = .2;
    const std::vector<std::array<Vector3d, 2>> edges{
        {Vector3d::Zero(), Vector3d::UnitX()},
        {Vector3d::Zero(), -Vector3d::UnitX()},
        {Vector3d::Zero(), Vector3d::UnitY()},
        {Vector3d::Zero(), -Vector3d::UnitY()}};
    const Vector3d p(.17, .12, .21);
    const auto sample = rod_joint_radius_sample(p, edges, radius, .25);
    for (int j = 0; j < 3; ++j) {
        const Vector3d delta = 1e-6 * radius * Vector3d::Unit(j);
        const double derivative =
            (rod_joint_radius_sample(p + delta, edges, radius, .25).residual -
             rod_joint_radius_sample(p - delta, edges, radius, .25).residual) /
            2e-6;
        CHECK(std::abs(derivative - sample.gradient[j]) < 1e-8);
    }
    const Vector3d shift(102, -34, 87);
    auto shifted = edges;
    for (auto& edge : shifted)
        for (auto& v : edge) v = 3 * v + shift;
    const auto transformed = rod_joint_radius_sample(3 * p + shift, shifted, 3 * radius, .25);
    CHECK(std::abs(transformed.residual - sample.residual) < 1e-12);
    CHECK((transformed.gradient - sample.gradient).norm() < 1e-12);
    const auto sharp = rod_joint_radius_sample(p, edges, radius, 0);
    CHECK(sample.residual < sharp.residual);
}
