#include <Eigen/Geometry>
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <wmtk/components/prismatic_mesh/junction_protection.hpp>
#include <wmtk/components/prismatic_mesh/prism_jacobian.hpp>
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

TEST_CASE("free rod ends receive terminal frames and prisms without frozen tet ports", "[rod]")
{
    auto input = tube();
    const auto positions = input.vertices;
    prepare_rod_mesh(input, options());
    const auto& report = input.rod_state->report;
    REQUIRE(report["joints"] == 0);
    REQUIRE(report["endpoints"] == 2);
    REQUIRE(report["free_endpoints"] == 2);
    REQUIRE(report["rods"] == 1);
    REQUIRE(report["target_degrees"][0] == 6);
    REQUIRE(report["matched_column_candidates"].get<size_t>() > 0);
    REQUIRE(report["tau_after"]["invalid_correspondence"] == 0);
    REQUIRE(report["tau_after"]["ordinary_tau22"] == 0);
    auto hybrid = build_prism_dominant_mesh(input, 0);
    REQUIRE(hybrid.report["prisms"] == 36);
    REQUIRE(hybrid.report["band_tetrahedra"] == 0);
    REQUIRE(input.rod_state->protected_input.empty());
    REQUIRE(input.rod_state->frozen_offset.empty());
    REQUIRE(input.junction_protection->centers.empty());
    REQUIRE(input.rod_state->vertex_frame.size() == 7);
    for (size_t v : input.rod_state->endpoints)
        REQUIRE(input.rod_state->frames.at(input.rod_state->vertex_frame.at(v)).valid);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
    for (size_t v : input.input_vertices) REQUIRE(input.vertices.row(v) == positions.row(v));
}

TEST_CASE("a single-edge free rod can be entirely prismatic", "[rod]")
{
    auto input = tube(2);
    prepare_rod_mesh(input, options());
    REQUIRE(input.rod_state->vertex_frame.size() == 2);
    REQUIRE(input.rod_state->frozen_offset.empty());
    const auto hybrid = build_prism_dominant_mesh(input, 0);
    REQUIRE(hybrid.report["prisms"] == 6);
    REQUIRE(hybrid.report["band_tetrahedra"] == 0);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
}

TEST_CASE("a regular rod vertex without an owning edge link recovers its material frame", "[rod]")
{
    constexpr size_t sections = 7, degree = 6, stride = degree + 1;
    auto input = tube(sections, degree);
    std::vector<Tet> cells(input.tetrahedra.rows());
    for (size_t i = 0; i < cells.size(); ++i)
        for (int j = 0; j < 4; ++j) cells[i][j] = input.tetrahedra(i, j);
    // Reverse one interval's pulling order. Its edge link now belongs to the
    // lower vertex, so section 2 has no edge link of its own on either side.
    const size_t u = stride, v = 2 * stride;
    for (size_t k = 0; k < degree; ++k) {
        const size_t a = u + k + 1, b = u + (k + 1) % degree + 1;
        const size_t A = v + k + 1, B = v + (k + 1) % degree + 1;
        cells[3 * degree + 3 * k] = {a, b, u, v};
        cells[3 * degree + 3 * k + 1] = {a, b, v, B};
        cells[3 * degree + 3 * k + 2] = {a, A, B, v};
    }
    for (const auto& t : cells) REQUIRE(tet_volume_above_threshold(input.vertices, t, 0));
    rebuild_fixture(input, cells);
    prepare_rod_mesh(input, options());
    REQUIRE(input.rod_state->vertex_frame.size() == sections);
    REQUIRE(input.rod_state->vertex_frame.count(v));
    const auto hybrid = build_prism_dominant_mesh(input, 0);
    REQUIRE(hybrid.report["prisms"] == (sections - 1) * degree);
    REQUIRE(hybrid.report["band_tetrahedra"] == 0);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
}

TEST_CASE(
    "rod frame construction collapses redundant fibers before prism reconstruction",
    "[rod][mixed]")
{
    constexpr size_t degree = 6, stride = degree + 1;
    auto input = tube(4, degree);
    const size_t u = stride, v = 2 * stride, end = 3 * stride;
    std::vector<Tet> cells;
    for (int i = 0; i < input.tetrahedra.rows(); ++i) {
        Tet t;
        for (int k = 0; k < 4; ++k) t[k] = input.tetrahedra(i, k);
        const bool has_u = std::find(t.begin(), t.end(), u) != t.end();
        const bool has_v = std::find(t.begin(), t.end(), v) != t.end();
        if (has_u && has_v) continue;
        for (auto& w : t)
            if (w == v) w = u;
        REQUIRE(tet_volume_above_threshold(input.vertices, t, 0));
        cells.push_back(t);
    }
    for (size_t w : input.input_to_offset_vertices[v]) {
        input.corr_input_vertex[w] = u;
        input.corr_input_vid[w] = input.source_vertex_ids[u];
        input.input_to_offset_vertices[u].push_back(w);
    }
    input.input_to_offset_vertices[v].clear();
    input.source_edges = {{0, u}, {u, end}};
    input.input_cells.assign(cells.size(), 0);
    input.offset_tet_tags.assign(cells.size(), 1);
    rebuild_fixture(input, cells);
    preprocess_rod_mesh(input, options());
    label_offset_faces(input);
    evaluate_target_positions(input, .1);
    construct_rod_frame_topology(input);
    const auto& report = input.rod_state->report["frame_topology_construction"];
    INFO(report.dump(2));
    REQUIRE(report["fiber_tets_before"].get<size_t>() > 0);
    REQUIRE(report["fiber_tets_after"] == 0);
    for (const auto& f : input.rod_state->frames)
        if (f.valid) REQUIRE(f.ring.size() == degree);
    const auto hybrid = build_prism_dominant_mesh(input, 0);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
}

TEST_CASE(
    "rod frame construction aligns an unequal annular triangulation with its background",
    "[rod][mixed]")
{
    constexpr size_t degree = 8, stride = degree + 1;
    auto input = tube(2, degree);
    for (size_t k = 0; k < degree; ++k) {
        const double angle = 2 * pi * (k + 1) / degree;
        input.vertices.row(stride + k + 1) << .2 * std::cos(angle), .2 * std::sin(angle), 1;
    }
    std::vector<Tet> cells;
    for (size_t k = 0; k < degree; ++k)
        cells.push_back({0, stride, stride + k + 1, stride + (k + 1) % degree + 1});
    // Equal-degree cyclic sections, but a U U U V V V strip is not the product
    // triangulation. Its missing vertical edge must be constructed explicitly.
    const std::vector<int> steps{0, 0, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1};
    size_t i = 0, j = 0;
    std::vector<std::array<size_t, 3>> front;
    for (size_t k = 0; k < steps.size(); ++k) {
        std::array<size_t, 3> f;
        if (steps[k] == 0) {
            f = {i % degree + 1, (i + 1) % degree + 1, stride + j % degree + 1};
            ++i;
        } else {
            f = {i % degree + 1, stride + (j + 1) % degree + 1, stride + j % degree + 1};
            ++j;
        }
        front.push_back(f);
        cells.push_back({0, f[0], f[1], f[2]});
    }
    bool obstructed = false;
    SECTION("a feasible cavity constructs all prism sectors") {}
    SECTION("an obstructed cavity preserves its original tets")
    {
        obstructed = true;
    }
    input.input_cells.assign(cells.size(), 0);
    input.offset_tet_tags.assign(cells.size(), 1);
    if (!obstructed) {
        const size_t p = input.vertices.rows();
        input.vertices.conservativeResize(p + 1, 3);
        Vector3d center = Vector3d::Zero(), normal = Vector3d::Zero();
        std::set<size_t> used;
        for (size_t k = 1; k <= 4; ++k) {
            const auto& f = front[k];
            for (size_t v : f) used.insert(v);
            const Vector3d a = input.vertices.row(f[0]), b = input.vertices.row(f[1]),
                           c = input.vertices.row(f[2]);
            normal += (b - a).cross(c - a);
        }
        for (size_t v : used) center += input.vertices.row(v).transpose() / used.size();
        input.vertices.row(p) = (center + .4 * normal.normalized()).transpose();
        input.vertex_tags.push_back(-1);
        input.source_vertex_ids.push_back(input.source_vertex_ids.back() + 7);
        input.corr_input_vertex.push_back(-1);
        input.corr_input_vid.push_back(-1);
        input.input_to_offset_vertices.resize(p + 1);
        for (size_t k = 1; k <= 4; ++k) {
            const auto& f = front[k];
            cells.push_back({p, f[0], f[2], f[1]});
            input.input_cells.push_back(0);
            input.offset_tet_tags.push_back(-1);
        }
    } else {
        std::map<size_t, size_t> outer;
        for (size_t center : {size_t(0), stride})
            for (size_t k = 0; k < degree; ++k) {
                const size_t v = center + k + 1, p = input.vertices.rows();
                outer[v] = p;
                input.vertices.conservativeResize(p + 1, 3);
                input.vertices.row(p) =
                    input.vertices.row(v).cwiseProduct(Eigen::RowVector3d(2, 2, 1));
                input.vertex_tags.push_back(-1);
                input.source_vertex_ids.push_back(input.source_vertex_ids.back() + 7);
                input.corr_input_vertex.push_back(-1);
                input.corr_input_vid.push_back(-1);
            }
        input.input_to_offset_vertices.resize(input.vertices.rows());
        input.input_cells.assign(cells.size(), 0);
        input.offset_tet_tags.assign(cells.size(), 1);
        for (const auto& f : front) {
            std::array<size_t, 3> order{0, 1, 2};
            std::sort(order.begin(), order.end(), [&](size_t a, size_t b) { return f[a] < f[b]; });
            const auto a = order[0], b = order[1], c = order[2];
            int parity = 0;
            for (int x = 0; x < 3; ++x)
                for (int y = x + 1; y < 3; ++y) parity += order[x] > order[y];
            for (auto t : std::vector<Tet>{
                     {f[a], f[b], f[c], outer.at(f[c])},
                     {f[a], f[b], outer.at(f[c]), outer.at(f[b])},
                     {f[a], outer.at(f[a]), outer.at(f[b]), outer.at(f[c])}}) {
                if (parity % 2) std::swap(t[0], t[1]);
                cells.push_back(t);
                input.input_cells.push_back(0);
                input.offset_tet_tags.push_back(-1);
            }
        }
    }
    for (const auto& t : cells) REQUIRE(tet_volume_above_threshold(input.vertices, t, 0));
    rebuild_fixture(input, cells);
    const auto original = input.vertices;
    preprocess_rod_mesh(input, options());
    label_offset_faces(input);
    evaluate_target_positions(input, .1);
    construct_rod_frame_topology(input);
    INFO(input.rod_state->report.dump(2));
    REQUIRE(input.vertices.topRows(original.rows()) == original);
    if (obstructed) {
        REQUIRE(input.rod_state->report["annulus_diagonal_exchanges"]["applied"] == 0);
        REQUIRE(
            input.rod_state->report["annulus_diagonal_exchanges"]["rejected"].get<size_t>() > 0);
        REQUIRE(input.tetrahedra.rows() == static_cast<int>(cells.size()));
        for (size_t t = 0; t < cells.size(); ++t)
            for (int k = 0; k < 4; ++k) REQUIRE(input.tetrahedra(t, k) == cells[t][k]);
    } else {
        REQUIRE(input.rod_state->report["annulus_diagonal_exchanges"]["applied"].get<size_t>() > 0);
        REQUIRE(input.rod_state->report["matched_column_candidates"] == degree);
    }
    const auto hybrid = build_prism_dominant_mesh(input, 0);
    if (!obstructed) {
        REQUIRE(hybrid.report["prisms"] == degree);
        REQUIRE(hybrid.report["band_tetrahedra"] == 0);
    }
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
}

TEST_CASE("rod frame construction respects generic layered-junction protection", "[rod][mixed]")
{
    auto input = tube();
    input.layered_junction_vertices.insert(21);
    preprocess_rod_mesh(input, options());
    REQUIRE_FALSE(input.junction_protection->offset_vertices.empty());
    size_t protected_frames = 0;
    for (const auto& frame : input.rod_state->frames) {
        const bool protected_ring =
            std::any_of(frame.ring.begin(), frame.ring.end(), [&](size_t v) {
                return input.junction_protection->offset_vertices.count(v);
            });
        if (!protected_ring) continue;
        ++protected_frames;
        REQUIRE_FALSE(frame.valid);
    }
    REQUIRE(protected_frames > 0);
    OptimizationOptions optimization;
    optimization.iterations = 2;
    optimization.keep_background_mesh = true;
    REQUIRE_NOTHROW(prism_main(input, .1, optimization));
    REQUIRE_NOTHROW(verify_junction_one_rings(input));
    for (const auto& [v, point] : input.junction_protection->offset_positions)
        REQUIRE(input.vertices.row(v) == point.transpose());
}

TEST_CASE("free terminal frames replace offset cone caps by flat prismatic ends", "[rod]")
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
            Tet t{center, center + k + 1, center + (k + 1) % degree + 1, apex};
            if (!tet_volume_above_threshold(input.vertices, t, 0)) std::swap(t[0], t[1]);
            REQUIRE(tet_volume_above_threshold(input.vertices, t, 0));
            cells.push_back(t);
        }
    }
    input.input_cells.resize(cells.size(), 0);
    input.offset_tet_tags.resize(cells.size(), 1);
    rebuild_fixture(input, cells);
    auto opt = options();
    opt.iterations = 0;
    prepare_rod_mesh(input, opt);
    auto hybrid = build_prism_dominant_mesh(input, 0);
    REQUIRE(hybrid.report["band_tetrahedra"] == 2 * degree);
    const auto positions = input.vertices.eval();
    const auto topology = input.tetrahedra.eval();
    SECTION("complete terminal disks end at real prism triangular faces")
    {
        finalize_rod_terminal_frames(input, hybrid);
        REQUIRE(input.vertices == positions);
        REQUIRE(input.tetrahedra == topology);
        REQUIRE(hybrid.report["prisms"] == (sections - 1) * degree);
        REQUIRE(hybrid.report["band_tetrahedra"] == 0);
        REQUIRE(hybrid.report["background_tetrahedra"] == 2 * degree);
        REQUIRE(input.rod_state->report["completed_prismatic_free_ends"] == 2);
        REQUIRE(input.rod_state->terminal_prism_cells.size() == 2 * degree);
        for (size_t ci : input.rod_state->terminal_prism_cells)
            REQUIRE(hybrid.cells[ci].type == HybridCellType::Prism);
        REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
        wmtk::MeshMap output;
        write_prismatic_mesh(input, "mesh.vtu", &hybrid, false, &output);
        REQUIRE(output.at("mesh.vtu").cells.size() == 1);
        REQUIRE(
            output.at("mesh.vtu").cell(wmtk::CellType::Prism).connectivity.rows() ==
            (sections - 1) * degree);
        // An already completed endpoint is safe to finalize again.
        finalize_rod_terminal_frames(input, hybrid);
        REQUIRE(hybrid.report["band_tetrahedra"] == 0);
        REQUIRE(input.rod_state->report["completed_prismatic_free_ends"] == 2);
    }
    SECTION("a missing prism sector cannot silently hide the offset cap")
    {
        auto& state = *input.rod_state;
        auto& ring = state.frames.at(state.vertex_frame.at(0)).ring;
        std::swap(ring[1], ring[2]); // The proposed fan no longer matches the actual prisms.
        finalize_rod_terminal_frames(input, hybrid);
        REQUIRE(hybrid.report["band_tetrahedra"] == degree);
        REQUIRE(state.report["completed_prismatic_free_ends"] == 1);
        REQUIRE(state.report["terminal_frames"][0]["status"] == "unmatched_terminal_prism_sector");
        REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
    }
}

TEST_CASE("only rod ends attached to non-rod structure retain one-rings", "[rod][mixed]")
{
    auto input = tube();
    constexpr size_t first = 0, last = 42;
    std::set<size_t> attached{first};
    attach_shell_fixture(input, first, -1);
    SECTION("one attached end and one free end") {}
    SECTION("both ends attached")
    {
        attach_shell_fixture(input, last, 1);
        attached.insert(last);
    }
    preprocess_rod_mesh(input, options());
    const auto& state = *input.rod_state;
    REQUIRE(state.input_edges.size() == 6);
    REQUIRE(state.protected_input == attached);
    REQUIRE(input.junction_protection->centers == attached);
    for (size_t v : {first, last}) {
        REQUIRE(state.free_endpoints.count(v) == (attached.count(v) == 0));
        if (!attached.count(v)) REQUIRE(state.vertex_frame.count(v));
    }
    const auto& path = state.report["rod_paths"][0];
    REQUIRE(path["start_kind"] == "non_rod_attachment");
    REQUIRE(path["end_kind"] == (attached.count(last) ? "non_rod_attachment" : "free_end"));
    OptimizationOptions optimization;
    optimization.iterations = 2;
    optimization.keep_background_mesh = true;
    prism_main(input, .1, optimization);
    REQUIRE_NOTHROW(verify_junction_one_rings(input));
    const auto hybrid = build_prism_dominant_mesh(input, 0);
    REQUIRE(hybrid.report["prisms"].get<size_t>() > 0);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
}

TEST_CASE(
    "rod winding follows path direction when vertex numbering reverses the traversal",
    "[rod]")
{
    auto input = tube();
    reverse_fixture_vertex_order(input);
    prepare_rod_mesh(input, options());
    const auto& s = *input.rod_state;
    REQUIRE(s.rods.size() == 1);
    const auto& path = s.rods.front();
    for (size_t k = 0; k < path.size(); ++k) {
        const size_t e = path[k];
        const auto& directed = s.directed_input_edges[e];
        if (k > 0) REQUIRE(s.directed_input_edges[path[k - 1]][1] == directed[0]);
        const auto& f = s.frames[e];
        REQUIRE(f.order == k);
        REQUIRE(s.report["rod_paths"][0]["edges"][k]["edge_order"] == k);
        REQUIRE(
            s.report["rod_paths"][0]["edges"][k]["from_vid"] ==
            input.source_vertex_ids[directed[0]]);
        if (!f.valid) continue;
        REQUIRE(f.tangent.z() < -.99);
        Vector3d area = Vector3d::Zero();
        const Vector3d center = input.vertices.row(f.center).transpose();
        for (size_t j = 0; j < f.ring.size(); ++j) {
            const Vector3d a = input.vertices.row(f.ring[j]).transpose();
            const Vector3d b = input.vertices.row(f.ring[(j + 1) % f.ring.size()]).transpose();
            area += (a - center).cross(b - center);
        }
        REQUIRE(area.dot(f.tangent) > 0);
    }
    auto hybrid = build_prism_dominant_mesh(input, 0);
    REQUIRE(hybrid.report["prisms"].get<size_t>() > 0);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
}

TEST_CASE("rod midpoint construction creates both prism layers and new correspondence", "[rod]")
{
    auto input = tube();
    prepare_rod_mesh(input, options());
    auto hybrid = build_prism_dominant_mesh(input, 0);
    const auto original_prisms = hybrid.report["prisms"].get<size_t>();
    const size_t original_nodes = input.vertices.rows();
    split_rod_intervals(input, hybrid);
    INFO(input.rod_state->report.dump(2));
    REQUIRE(input.rod_state->report["axially_split_intervals"].get<size_t>() > 0);
    REQUIRE(hybrid.report["prisms"] == 2 * original_prisms);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
    size_t new_inputs = 0, new_offsets = 0;
    for (size_t v = original_nodes; v < static_cast<size_t>(input.vertices.rows()); ++v) {
        if (input.vertex_tags[v] == 1) {
            ++new_inputs;
            REQUIRE(input.corr_input_vid[v] == -1);
        } else {
            ++new_offsets;
            REQUIRE(input.corr_input_vertex[v] >= static_cast<int64_t>(original_nodes));
            REQUIRE(input.corr_input_vid[v] == input.source_vertex_ids[input.corr_input_vertex[v]]);
            const Vector3d delta = input.vertices.row(v).transpose() -
                                   input.vertices.row(input.corr_input_vertex[v]).transpose();
            REQUIRE(std::abs(delta.norm() - .2) < 1e-12);
            REQUIRE(std::abs(delta.z()) < 1e-12);
        }
    }
    REQUIRE(new_inputs > 0);
    REQUIRE(new_offsets == 6 * new_inputs);
    REQUIRE(analyze_rod_tau(input)["edge_parent_mismatch"] == 0);
    wmtk::MeshMap output;
    REQUIRE_NOTHROW(write_prismatic_mesh(input, "mesh.vtu", &hybrid, true, &output));
    REQUIRE(output.count("mesh.vtu") == 1);
}

TEST_CASE("degree-two right-angle corners attempt prisms without creating joint ports", "[rod]")
{
    constexpr size_t sections = 7, degree = 6, corner_section = 3;
    auto input = tube(sections, degree);
    const size_t corner = corner_section * (degree + 1);
    for (size_t i = 0; i < sections; ++i) {
        const Vector3d center =
            i <= corner_section
                ? Vector3d(0, 0, static_cast<double>(i))
                : Vector3d(static_cast<double>(i - corner_section), 0, corner_section);
        const Vector3d d1 = i < corner_section    ? Vector3d::UnitX().eval()
                            : i == corner_section ? Vector3d(1, 0, -1).normalized()
                                                  : -Vector3d::UnitZ().eval();
        const size_t v = i * (degree + 1);
        input.vertices.row(v) = center.transpose();
        for (size_t k = 0; k < degree; ++k)
            input.vertices.row(v + k + 1) =
                (center + .2 * (std::cos(2 * pi * k / degree) * d1 +
                                std::sin(2 * pi * k / degree) * Vector3d::UnitY()))
                    .transpose();
    }
    const auto positions = input.vertices;
    prepare_rod_mesh(input, options());
    const auto& s = *input.rod_state;
    REQUIRE(s.report["joints"] == 0);
    REQUIRE(s.report["endpoints"] == 2);
    REQUIRE(s.report["rods"] == 1);
    REQUIRE_FALSE(s.protected_input.count(corner));
    REQUIRE(s.vertex_frame.count(corner) == 1);
    for (size_t e = 0; e < s.frames.size(); ++e) {
        const auto& f = s.frames[e];
        if (!f.valid) continue;
        REQUIRE(f.rotation.angularDistance(s.vertex_rotations.at(f.center)) < 1e-12);
        REQUIRE((f.tangent - f.rotation * Vector3d::UnitZ()).norm() < 1e-12);
    }
    const auto normal = s.vertex_rotations.at(corner) * Vector3d::UnitZ();
    REQUIRE(normal.dot(Vector3d::UnitZ()) > .7);
    REQUIRE(normal.dot(Vector3d::UnitX()) > .7);
    REQUIRE(std::any_of(s.columns.begin(), s.columns.end(), [&](const auto& column) {
        return column[0] == corner || column[3] == corner;
    }));
    auto hybrid = build_prism_dominant_mesh(input, 0);
    REQUIRE(std::any_of(hybrid.cells.begin(), hybrid.cells.end(), [&](const HybridCell& cell) {
        return cell.type == HybridCellType::Prism &&
               (cell.vertices[0] == corner || cell.vertices[3] == corner);
    }));
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
    for (size_t v : input.input_vertices) REQUIRE(input.vertices.row(v) == positions.row(v));
}

TEST_CASE("every input vertex uses its incident edge quaternion mean", "[rod]")
{
    RodMeshState state;
    state.directed_input_edges = {{{0, 1}}, {{1, 2}}};
    state.edge_frames.resize(2);
    const Eigen::Quaterniond a =
        Eigen::AngleAxisd(.31, Vector3d::UnitZ()) * Eigen::AngleAxisd(-.9, Vector3d::UnitY());
    const Eigen::Quaterniond b =
        Eigen::AngleAxisd(1.1, Vector3d::UnitY()) * Eigen::AngleAxisd(.27, Vector3d::UnitZ());
    state.edge_frames[0].rotation = a;
    state.edge_frames[1].rotation = b;
    average_rod_vertex_frames(state);
    REQUIRE(state.vertex_rotations.size() == 3);
    REQUIRE(state.vertex_rotations.at(0).angularDistance(a) < 1e-12);
    REQUIRE(state.vertex_rotations.at(2).angularDistance(b) < 1e-12);
    REQUIRE(state.vertex_rotations.at(1).angularDistance(a.slerp(.5, b)) < 1e-12);
    state.edge_frames[1].rotation.coeffs() *= -1;
    average_rod_vertex_frames(state);
    REQUIRE(state.vertex_rotations.at(1).angularDistance(a.slerp(.5, b)) < 1e-12);
    state.directed_input_edges.push_back({1, 3});
    state.edge_frames.push_back({a, 0, false});
    average_rod_vertex_frames(state);
    const auto joint = state.vertex_rotations.at(1);
    state.edge_frames.back().rotation.coeffs() *= -1;
    average_rod_vertex_frames(state);
    REQUIRE(state.vertex_rotations.at(1).angularDistance(joint) < 1e-12);
}

TEST_CASE("rod tau analysis catches same-count but incorrect midpoint parents", "[rod]")
{
    auto input = tube();
    prepare_rod_mesh(input, options());
    const size_t offset = 15;
    REQUIRE(input.vertex_tags[offset] == 2);
    input.corr_input_vertex[offset] = input.input_vertices.back();
    input.corr_input_vid[offset] = input.source_vertex_ids[input.input_vertices.back()];
    REQUIRE(analyze_rod_tau(input)["invalid_correspondence"] == 0);
    REQUIRE(analyze_rod_tau(input)["edge_parent_mismatch"].get<size_t>() > 0);
}

TEST_CASE(
    "quaternion corner buffers keep the incident-edge mean and both interfaces fixed",
    "[rod]")
{
    constexpr size_t degree = 6, sections = 7, corner_section = 3;
    auto source = tube(sections, degree);
    for (size_t i = 0; i < sections; ++i) {
        const Vector3d c = i <= corner_section ? Vector3d(0, 0, i)
                                               : Vector3d(i - corner_section, 0, corner_section);
        const Vector3d x = i < corner_section    ? Vector3d::UnitX().eval()
                           : i == corner_section ? Vector3d(1, 0, -1).normalized()
                                                 : -Vector3d::UnitZ().eval();
        const size_t v = i * (degree + 1);
        source.vertices.row(v) = c.transpose();
        for (size_t k = 0; k < degree; ++k)
            source.vertices.row(v + k + 1) =
                (c + .2 * (std::cos(2 * pi * k / degree) * x +
                           std::sin(2 * pi * k / degree) * Vector3d::UnitY()))
                    .transpose();
    }
    prepare_rod_mesh(source, options());
    const size_t corner = corner_section * (degree + 1);
    const int64_t corner_vid = source.source_vertex_ids[corner];
    auto& frame = source.rod_state->frames[source.rod_state->vertex_frame.at(corner)];
    // A common material twist preserves averaging and exercises the full pose.
    for (auto& edge : source.rod_state->edge_frames)
        edge.rotation = edge.rotation * Eigen::AngleAxisd(.17, Vector3d::UnitZ());
    average_rod_vertex_frames(*source.rod_state);
    for (auto& f : source.rod_state->frames) {
        if (!f.valid) continue;
        f.rotation = source.rod_state->vertex_rotations.at(f.center);
        f.tangent = f.rotation * Vector3d::UnitZ();
    }
    const Eigen::Quaterniond fixed = source.rod_state->vertex_rotations.at(corner);
    const Vector3d c = source.vertices.row(corner).transpose();
    for (size_t k = 0; k < degree; ++k)
        source.vertices.row(frame.ring[k]) =
            (c + .2 * (fixed *
                       Vector3d(std::cos(2 * pi * k / degree), std::sin(2 * pi * k / degree), 0)))
                .transpose();
    SECTION("existing local frame") {}
    SECTION("missing selected ring still uses the incident-edge frame mean")
    {
        source.rod_state->vertex_frame.erase(corner);
    }
    const auto original_vertices = source.vertices;
    const auto original_tets = source.tetrahedra;
    const auto original_report = source.rod_state->report;
    const auto patch = construct_rod_corner_buffer(source, corner_vid, 15);
    const auto& r = patch.input.rod_state->report;
    REQUIRE(r["status"] == "prism_buffer");
    REQUIRE(r["anchor_source"] == "averaged_incident_edge_frames");
    REQUIRE(r["corner_position_unchanged"] == true);
    REQUIRE(r["outer_interfaces_unchanged"] == true);
    REQUIRE(r["anchor_orientation_error_radians"].get<double>() < 1e-14);
    REQUIRE(r["maximum_frame_angle_degrees"].get<double>() <= 15 + 1e-10);
    const auto& qv = r["fixed_corner_quaternion_wxyz"];
    const Eigen::Quaterniond anchor(
        qv[0].get<double>(),
        qv[1].get<double>(),
        qv[2].get<double>(),
        qv[3].get<double>());
    REQUIRE(((anchor * Vector3d::UnitZ()) - frame.tangent).norm() < 1e-12);
    REQUIRE(anchor.angularDistance(fixed) < 1e-12);
    REQUIRE(r["background_integrated"] == false);
    REQUIRE(source.vertices == original_vertices);
    REQUIRE(source.tetrahedra == original_tets);
    REQUIRE(source.rod_state->report == original_report);
    REQUIRE(
        patch.hybrid.cells.size() ==
        degree * (r["left_layers"].get<size_t>() + r["right_layers"].get<size_t>()));
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(patch.input, patch.hybrid, 0));
    REQUIRE(r["tau_after"]["ordinary_tau22"] == 0);
    for (const auto& f : {r["frames"].front(), r["frames"].back()})
        for (const auto& id : f["ring_vids"]) {
            const auto find = [&](const auto& input) {
                return std::find(
                           input.source_vertex_ids.begin(),
                           input.source_vertex_ids.end(),
                           id.get<int64_t>()) -
                       input.source_vertex_ids.begin();
            };
            REQUIRE(
                patch.input.vertices.row(find(patch.input)) == source.vertices.row(find(source)));
        }
    REQUIRE_THROWS(construct_rod_corner_buffer(source, corner_vid, 0));
    REQUIRE_THROWS(construct_rod_corner_buffer(source, corner_vid, 1e-300));
    REQUIRE_THROWS(
        construct_rod_corner_buffer(source, corner_vid, std::numeric_limits<double>::quiet_NaN()));
    REQUIRE_THROWS(construct_rod_corner_buffer(source, -1, 15));
    REQUIRE_THROWS(construct_rod_corner_buffer(source, source.source_vertex_ids.front(), 15));
}

TEST_CASE("an infeasible midpoint frame rolls back the whole local proposal", "[rod]")
{
    auto input = tube();
    prepare_rod_mesh(input, options());
    auto hybrid = build_prism_dominant_mesh(input, 0);
    const auto vertices = input.vertices;
    const auto tets = input.tetrahedra;
    const auto ids = input.source_vertex_ids;
    const auto corr = input.corr_input_vid;
    const auto count = hybrid.report["prisms"];
    // Existing reference volumes are ~0.00577; midpoint halves are ~0.00289.
    input.rod_state->options.min_tet_volume = .004;
    split_rod_intervals(input, hybrid);
    REQUIRE(input.rod_state->report["axially_split_intervals"] == 0);
    REQUIRE(input.rod_state->report["axial_split_rejected_intervals"].get<size_t>() > 0);
    REQUIRE(input.vertices == vertices);
    REQUIRE(input.tetrahedra == tets);
    REQUIRE(input.source_vertex_ids == ids);
    REQUIRE(input.corr_input_vid == corr);
    REQUIRE(hybrid.report["prisms"] == count);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, .004));
}

TEST_CASE("corner subdivision can recover a prism after a coarser attempt fails", "[rod]")
{
    constexpr size_t degree = 3, stride = degree + 1;
    auto source = tube(5, degree);
    constexpr double radius = .4058111976714778;
    const Eigen::Quaterniond ql(
        .8241248836128083,
        -.4197137615772836,
        .34859958658701906,
        .15210806285066497);
    const Eigen::Quaterniond qc(
        -.7682894865247487,
        .31907384739052047,
        -.3725360706232039,
        -.41126636246059817);
    const Eigen::Quaterniond qr(
        .6225504037833303,
        .19327701127411745,
        .2095260082267239,
        .7288167420818021);
    const std::array<Vector3d, 5> centers{{{0, 0, -1}, {0, 0, 0}, {0, 0, 1}, {1, 0, 1}, {2, 0, 1}}};
    const std::array<Eigen::Quaterniond, 5> rotations{{ql, ql, qc, qr, qr}};
    for (size_t i = 0; i < centers.size(); ++i) {
        source.vertices.row(i * stride) = centers[i].transpose();
        for (size_t k = 0; k < degree; ++k)
            source.vertices.row(i * stride + k + 1) =
                (centers[i] +
                 radius *
                     (rotations[i].normalized() *
                      Vector3d(std::cos(2 * pi * k / degree), std::sin(2 * pi * k / degree), 0)))
                    .transpose();
    }
    // This source has a valid tetrahedral subdivision even though its coarse
    // six-node prism mapping folds. Use the opposite outer-face diagonal.
    std::vector<Tet> tets;
    for (size_t i = 1; i < centers.size(); ++i) {
        const size_t u = (i - 1) * stride, v = i * stride;
        for (size_t k = 0; k < degree; ++k) {
            const size_t a = u + k + 1, b = u + (k + 1) % degree + 1;
            const size_t A = v + k + 1, B = v + (k + 1) % degree + 1;
            for (const Tet t : std::array<Tet, 3>{{{b, u, a, A}, {b, u, A, B}, {v, u, B, A}}}) {
                REQUIRE(tet_volume_above_threshold(source.vertices, t, 0));
                tets.push_back(t);
            }
        }
    }
    rebuild_fixture(source, tets);
    source.rod_state = std::make_shared<RodMeshState>();
    auto& state = *source.rod_state;
    state.radius = state.options.radius = radius;
    state.protected_input = {0, 4 * stride};
    state.frames.resize(4);
    for (size_t i = 0; i < 4; ++i) {
        const std::array<size_t, 2> edge{i * stride, (i + 1) * stride};
        state.input_edges.push_back(edge);
        state.directed_input_edges.push_back(edge);
        auto& f = state.frames[i];
        f.edge = edge;
        f.center = edge[1];
        f.order = i;
        f.rotation = rotations[i + 1].normalized();
        state.vertex_rotations[f.center] = f.rotation;
        f.tangent = f.rotation * Vector3d::UnitZ();
        for (size_t k = 0; k < degree; ++k) f.ring.push_back(f.center + k + 1);
        f.valid = i < 3;
        if (f.valid) state.vertex_frame[f.center] = i;
    }
    state.rods = {{0, 1, 2, 3}};
    const auto original = source.vertices;
    const auto coarse =
        construct_rod_corner_buffer(source, source.source_vertex_ids[2 * stride], 90, 0);
    REQUIRE(coarse.input.rod_state->report["status"] == "tet_fallback");
    REQUIRE(coarse.input.rod_state->report["subdivision_attempts"].size() == 1);
    REQUIRE(coarse.hybrid.report["prisms"] == 0);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(coarse.input, coarse.hybrid, 0));
    const auto refined =
        construct_rod_corner_buffer(source, source.source_vertex_ids[2 * stride], 90, 1);
    const auto& report = refined.input.rod_state->report;
    REQUIRE(report["status"] == "prism_buffer");
    REQUIRE(report["subdivision_depth"] == 1);
    REQUIRE(report["subdivision_attempts"].size() == 2);
    REQUIRE(report["subdivision_attempts"][0]["valid"] == false);
    REQUIRE(report["subdivision_attempts"][1]["valid"] == true);
    REQUIRE(report["anchor_orientation_error_radians"].get<double>() < 1e-14);
    REQUIRE(refined.hybrid.report["prisms"] == 12);
    REQUIRE(source.vertices == original);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(refined.input, refined.hybrid, 0));
    REQUIRE_THROWS(
        construct_rod_corner_buffer(source, source.source_vertex_ids[2 * stride], 90, -1));
    REQUIRE_THROWS(
        construct_rod_corner_buffer(source, source.source_vertex_ids[2 * stride], 90, 8));
}

TEST_CASE(
    "a degenerate fixed corner frame stops at the subdivision limit and keeps valid tets",
    "[rod]")
{
    auto source = tube(7, 4);
    prepare_rod_mesh(source, options());
    const size_t corner = 3 * 5;
    auto& frame = source.rod_state->frames[source.rod_state->vertex_frame.at(corner)];
    // Keep the valid, nondegenerate source mesh, but supply the legacy edge-normal
    // corner pose. Its planar target contains the centerline, so J must vanish.
    const Vector3d center = source.vertices.row(corner).transpose();
    const auto first =
        std::max_element(frame.ring.begin(), frame.ring.end(), [&](size_t a, size_t b) {
            return source.vertices(a, 1) < source.vertices(b, 1);
        });
    std::rotate(frame.ring.begin(), first, frame.ring.end());
    source.vertices.row(frame.ring.front()) = (center + .2 * Vector3d::UnitY()).transpose();
    frame.tangent = Vector3d::UnitX();
    frame.rotation = Eigen::Quaterniond(.5, .5, .5, .5);
    source.rod_state->vertex_rotations[corner] = frame.rotation;
    const auto original = source.vertices;
    const auto patch = construct_rod_corner_buffer(source, source.source_vertex_ids[corner], 45, 2);
    const auto& report = patch.input.rod_state->report;
    REQUIRE(report["status"] == "tet_fallback");
    REQUIRE(report["subdivision_attempts"].size() == 3);
    REQUIRE(report["subdivision_attempts"].back()["depth"] == 2);
    for (const auto& a : report["subdivision_attempts"]) REQUIRE(a["valid"] == false);
    REQUIRE(source.vertices == original);
    REQUIRE(std::all_of(patch.hybrid.cells.begin(), patch.hybrid.cells.end(), [](const auto& c) {
        return c.type == HybridCellType::Tetrahedron;
    }));
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(patch.input, patch.hybrid, 0));
}

TEST_CASE("rod options reject invalid radii and surface complexes", "[rod]")
{
    SECTION("negative radius")
    {
        auto input = tube();
        auto opt = options();
        opt.radius = -1;
        REQUIRE_THROWS(prepare_rod_mesh(input, opt));
    }
    SECTION("nonfinite radius")
    {
        auto input = tube();
        auto opt = options();
        opt.radius = std::numeric_limits<double>::quiet_NaN();
        REQUIRE_THROWS(prepare_rod_mesh(input, opt));
    }
    SECTION("input triangle")
    {
        auto input = tube();
        input.vertex_tags[1] = 1;
        REQUIRE_THROWS(prepare_rod_mesh(input, options()));
    }
}

TEST_CASE(
    "mixed shell rod and solid share targets optimization and hybrid reconstruction",
    "[rod][mixed]")
{
    auto input = tube();
    const size_t original_rod_vertices = input.vertices.rows();
    std::vector<Tet> tets;
    for (int i = 0; i < input.tetrahedra.rows(); ++i) {
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(i, j);
        tets.push_back(t);
    }
    const auto vertex = [&](const Vector3d& x, int label, int64_t parent = -1) {
        const size_t v = input.vertices.rows();
        input.vertices.conservativeResize(v + 1, 3);
        input.vertices.row(v) = x.transpose();
        input.vertex_tags.push_back(label);
        input.source_vertex_ids.push_back(input.source_vertex_ids.back() + 7);
        input.corr_input_vertex.push_back(parent);
        input.corr_input_vid.push_back(parent < 0 ? -1 : input.source_vertex_ids[parent]);
        input.input_to_offset_vertices.resize(v + 1);
        if (label == 1) input.input_vertices.push_back(v);
        if (label == 2) {
            input.offset_vertices.push_back(v);
            input.input_to_offset_vertices[parent].push_back(v);
        }
        return v;
    };
    constexpr size_t n = 7;
    std::array<size_t, n * n> lower, upper;
    for (size_t j = 0; j < n; ++j)
        for (size_t i = 0; i < n; ++i) {
            const size_t k = j * n + i;
            lower[k] = vertex(Vector3d(3 + i, j, 0), 1);
            upper[k] = vertex(Vector3d(3 + i, j, .3), 2, lower[k]);
        }
    const auto cell = [&](Tet t, int in, int band) {
        if (!tet_volume_above_threshold(input.vertices, t, 0)) std::swap(t[0], t[1]);
        REQUIRE(tet_volume_above_threshold(input.vertices, t, 0));
        tets.push_back(t);
        input.input_cells.push_back(in);
        input.offset_tet_tags.push_back(band);
    };
    for (size_t j = 0; j + 1 < n; ++j)
        for (size_t i = 0; i + 1 < n; ++i) {
            const size_t a = j * n + i, b = a + 1, c = a + n, d = c + 1;
            for (auto tri : {std::array<size_t, 3>{a, b, d}, std::array<size_t, 3>{a, c, d}}) {
                const auto x = tri[0], y = tri[1], z = tri[2];
                cell({lower[x], lower[y], lower[z], upper[z]}, 0, 1);
                cell({lower[x], lower[y], upper[z], upper[y]}, 0, 1);
                cell({lower[x], upper[x], upper[y], upper[z]}, 0, 1);
            }
        }
    const Tet solid{
        vertex({-4, 0, 0}, 1),
        vertex({-3, 0, 0}, 1),
        vertex({-4, 1, 0}, 1),
        vertex({-4, 0, 1}, 1)};
    cell(solid, 1, -1);
    rebuild_fixture(input, tets);
    const auto positions = input.vertices;
    wmtk::MeshMap raw;
    write_prismatic_mesh(input, "mesh.vtu", nullptr, true, &raw);
    const auto result = prismatic_mesh(
        {{"application", "prismatic_mesh"},
         {"input", "construction"},
         {"output", "unused.vtu"},
         {"rod_mode", true},
         {"rod_radius", .16},
         {"rod_axial_split", true},
         {"iterations", 2},
         {"keep_background_mesh", true},
         {"prism_dominant", true}},
        {{"construction", raw.at("mesh.vtu")}});
    REQUIRE(result.second["rod"]["unified_optimization"] == true);
    REQUIRE(result.second["rod"]["input_edges"] == 6);
    REQUIRE(result.second["rod"]["shared_targets"]["rod_vertices"].get<size_t>() > 0);
    REQUIRE(result.second["rod"]["shared_targets"]["shell_vertices"].get<size_t>() > 0);
    REQUIRE(result.second["rod"]["axially_split_intervals"].get<size_t>() > 0);
    const auto band_only = prismatic_mesh(
        {{"application", "prismatic_mesh"},
         {"input", "construction"},
         {"output", "unused.vtu"},
         {"rod_mode", true},
         {"rod_radius", .16},
         {"iterations", 2},
         {"ignore_background_mesh", true},
         {"keep_background_mesh", true},
         {"prism_dominant", true}},
        {{"construction", raw.at("mesh.vtu")}});
    REQUIRE(band_only.second["hybrid"]["ignore_background_mesh"] == true);
    REQUIRE(band_only.second["hybrid"]["input_tetrahedra"] == 1);
    REQUIRE(band_only.second["hybrid"]["background_tetrahedra"] == 0);
    REQUIRE(band_only.second["hybrid"]["background_pyramids"] == 0);
    for (const auto& [name, mesh] : result.first) {
        const auto target = mesh.vertex_attributes.find("target_position");
        if (target != mesh.vertex_attributes.end()) REQUIRE(target->second.rows() == mesh.V.rows());
    }

    preprocess_rod_mesh(input, options());
    REQUIRE(input.vertices == positions); // preprocessing computes targets without moving them
    OptimizationOptions optimization;
    optimization.iterations = 2;
    optimization.keep_background_mesh = true;
    prism_main(input, .1, optimization);
    REQUIRE(input.rod_state->input_edges.size() == 6);
    size_t shell_moved = 0;
    for (size_t v : upper) shell_moved += input.vertices.row(v) != positions.row(v);
    REQUIRE(shell_moved > 0);
    for (size_t v : input.input_vertices) REQUIRE(input.vertices.row(v) == positions.row(v));
    const auto hybrid = build_prism_dominant_mesh(input, 0);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
    size_t shell_prisms = 0, rod_prisms = 0, solids = 0;
    for (const auto& c : hybrid.cells) {
        solids += input.input_cells[c.source_tets[0]] == 1;
        if (c.type != HybridCellType::Prism) continue;
        const size_t in = std::count_if(c.vertices.begin(), c.vertices.end(), [&](size_t v) {
            return input.vertex_tags[v] == 1;
        });
        shell_prisms += in == 3;
        rod_prisms += in == 2;
    }
    REQUIRE(shell_prisms > 0);
    REQUIRE(rod_prisms > 0);
    REQUIRE(solids == 1);
    for (size_t v : input.rod_state->retained_ring_vertices) {
        REQUIRE(v < original_rod_vertices);
        REQUIRE_FALSE(
            rod_collapse_allowed(input, v, *input.rod_state->retained_ring_vertices.begin()));
    }
}

TEST_CASE("shared rod smoothing checks the complete blocking background star", "[rod][mixed]")
{
    auto input = tube();
    const std::array<size_t, 3> face{15, 16, 23};
    const Vector3d a = input.vertices.row(face[0]), b = input.vertices.row(face[1]),
                   c = input.vertices.row(face[2]);
    Vector3d normal = (b - a).cross(c - a).normalized();
    if (normal.dot(Vector3d(input.vertices.row(14)) - a) > 0) normal = -normal;
    const size_t p = input.vertices.rows();
    input.vertices.conservativeResize(p + 1, 3);
    input.vertices.row(p) = ((a + b + c) / 3 + .0001 * normal).transpose();
    input.vertex_tags.push_back(-1);
    input.source_vertex_ids.push_back(input.source_vertex_ids.back() + 7);
    input.corr_input_vid.push_back(-1);
    input.corr_input_vertex.push_back(-1);
    input.input_to_offset_vertices.resize(p + 1);
    std::vector<Tet> cells;
    for (int i = 0; i < input.tetrahedra.rows(); ++i) {
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(i, j);
        cells.push_back(t);
    }
    Tet background{p, face[0], face[1], face[2]};
    if (!tet_volume_above_threshold(input.vertices, background, 0))
        std::swap(background[0], background[1]);
    cells.push_back(background);
    input.input_cells.push_back(0);
    input.offset_tet_tags.push_back(-1);
    rebuild_fixture(input, cells);
    const auto original_positions = input.vertices;
    const auto original_ids = input.source_vertex_ids;
    wmtk::MeshMap raw;
    write_prismatic_mesh(input, "mesh.vtu", nullptr, true, &raw);
    auto opt = options();
    opt.radius = .4;
    opt.iterations = 0;
    preprocess_rod_mesh(input, opt);
    OptimizationOptions optimization;
    optimization.iterations = 2;
    optimization.keep_background_mesh = true;
    prism_main(input, .1, optimization);
    REQUIRE(tet_volume_above_threshold(input.vertices, background, 0));
    const auto ignored = prismatic_mesh(
        {{"application", "prismatic_mesh"},
         {"input", "construction"},
         {"output", "unused.vtu"},
         {"rod_mode", true},
         {"rod_radius", .4},
         {"iterations", 2},
         {"ignore_background_mesh", true},
         {"keep_background_mesh", true},
         {"prism_dominant", true}},
        {{"construction", raw.at("mesh.vtu")}});
    const auto& exported = ignored.first.at("mesh");
    const auto ids = exported.vertex_attributes.at("vid").integers();
    auto unconstrained_positions = original_positions;
    for (int i = 0; i < exported.V.rows(); ++i) {
        const auto old = std::find(original_ids.begin(), original_ids.end(), ids(i, 0));
        if (old != original_ids.end())
            unconstrained_positions.row(old - original_ids.begin()) = exported.V.row(i);
    }
    REQUIRE(ignored.second["hybrid"]["background_tetrahedra"] == 0);
    // The old background tet would now invert: the new flag genuinely removed
    // its geometric constraint, rather than just hiding it from the output.
    REQUIRE_FALSE(tet_volume_above_threshold(unconstrained_positions, background, 0));
    for (size_t v : input.rod_state->frozen_offset) REQUIRE(input.singular_vertex_tags[v] == 1);
    for (int i = 0; i < input.tetrahedra.rows(); ++i) {
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(i, j);
        REQUIRE(tet_volume_above_threshold(input.vertices, t, 0));
    }
}

TEST_CASE("rod frames split to the maximum degree with valid new parent IDs", "[rod]")
{
    auto input = tube();
    refine_fixture_ring(input, 15, 16);
    auto opt = options();
    opt.iterations = 0;
    prepare_rod_mesh(input, opt);
    INFO(input.rod_state->report.dump(2));
    REQUIRE(input.rod_state->report["target_degrees"][0] == 7);
    REQUIRE(input.rod_state->report["ring_edge_splits"].get<size_t>() > 0);
    REQUIRE(input.rod_state->report["degree_alignment_blocked"] == 0);
    for (const auto& f : input.rod_state->frames)
        if (f.valid) REQUIRE(f.ring.size() == 7);
    REQUIRE(analyze_rod_tau(input)["invalid_correspondence"] == 0);
    for (int i = 0; i < input.tetrahedra.rows(); ++i) {
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(i, j);
        REQUIRE(tet_volume_above_threshold(input.vertices, t, 0));
    }
}

TEST_CASE("a blocked global degree target retains compatible existing rod sections", "[rod]")
{
    auto input = tube(5);
    refine_fixture_ring(input, 15, 16);
    auto opt = options();
    opt.iterations = 0;
    opt.min_tet_volume = .004; // Ring-edge split children would be smaller than the floor.
    prepare_rod_mesh(input, opt);
    const auto& state = *input.rod_state;
    REQUIRE(state.report["degree_alignment_blocked"].get<size_t>() > 0);
    REQUIRE(state.vertex_frame.count(0));
    REQUIRE(state.vertex_frame.count(7));
    REQUIRE(state.frames.at(state.vertex_frame.at(0)).ring.size() == 6);
    REQUIRE(state.frames.at(state.vertex_frame.at(7)).ring.size() == 6);
    REQUIRE(state.report["matched_column_candidates"].get<size_t>() > 0);
}

TEST_CASE("rod tau22 normalization preserves a valid tet fallback", "[rod]")
{
    auto input = tube(9, 6, true);
    auto opt = options();
    opt.iterations = 4;
    prepare_rod_mesh(input, opt);
    const auto& report = input.rod_state->report;
    INFO(report.dump(2));
    REQUIRE(report["tau_before"]["counts"]["tau22"].get<size_t>() > 0);
    REQUIRE(report["tau22_unlock_attempts"].get<size_t>() > 0);
    REQUIRE(report["tau_after"]["invalid_correspondence"] == 0);
    auto hybrid = build_prism_dominant_mesh(input, 0);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
    REQUIRE(hybrid.report["band_tetrahedra"].get<size_t>() > 0);
}

TEST_CASE("rod circular fitting respects a blocking background tetrahedron", "[rod]")
{
    auto input = tube();
    const std::array<size_t, 3> face{15, 16, 23};
    const Vector3d a = input.vertices.row(face[0]), b = input.vertices.row(face[1]),
                   c = input.vertices.row(face[2]);
    Vector3d normal = (b - a).cross(c - a).normalized();
    const Vector3d inner = input.vertices.row(14);
    if (normal.dot(inner - a) > 0) normal = -normal;
    const size_t p = input.vertices.rows();
    input.vertices.conservativeResize(p + 1, 3);
    input.vertices.row(p) = ((a + b + c) / 3 + .0001 * normal).transpose();
    input.vertex_tags.push_back(-1);
    input.source_vertex_ids.push_back(input.source_vertex_ids.back() + 7);
    input.corr_input_vid.push_back(-1);
    input.corr_input_vertex.push_back(-1);
    input.input_to_offset_vertices.resize(p + 1);
    std::vector<Tet> tets;
    for (int i = 0; i < input.tetrahedra.rows(); ++i) {
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(i, j);
        tets.push_back(t);
    }
    Tet background{p, face[0], face[1], face[2]};
    if (!tet_volume_above_threshold(input.vertices, background, 0))
        std::swap(background[0], background[1]);
    tets.push_back(background);
    input.input_cells.push_back(0);
    input.offset_tet_tags.push_back(-1);
    rebuild_fixture(input, tets);
    auto opt = options();
    opt.radius = .4;
    opt.iterations = 0;
    prepare_rod_mesh(input, opt);
    REQUIRE(input.rod_state->report["relocation_limited_frames"].get<size_t>() > 0);
    REQUIRE(tet_volume_above_threshold(input.vertices, background, 0));
}

TEST_CASE("closed rods form periodic frames without artificial endpoints", "[rod]")
{
    constexpr size_t sections = 12, degree = 6;
    auto input = tube(sections, degree);
    bool nonplanar = false;
    SECTION("planar loop") {}
    SECTION("nonplanar loop")
    {
        nonplanar = true;
    }
    for (size_t i = 0; i < sections; ++i) {
        const double theta = 2 * pi * i / sections;
        const Vector3d radial(std::cos(theta), std::sin(theta), 0);
        const Vector3d height =
            nonplanar ? Vector3d(0, 0, .12 * std::sin(2 * theta) + .08 * std::cos(3 * theta))
                      : Vector3d::Zero();
        const size_t center = i * (degree + 1);
        input.vertices.row(center) = (2 * radial + height).transpose();
        for (size_t k = 0; k < degree; ++k)
            input.vertices.row(center + k + 1) =
                (2 * radial + height + .2 * std::cos(2 * pi * k / degree) * radial -
                 .2 * std::sin(2 * pi * k / degree) * Vector3d::UnitZ())
                    .transpose();
    }
    std::vector<Tet> tets;
    for (int i = 0; i < input.tetrahedra.rows(); ++i) {
        Tet t;
        for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(i, j);
        tets.push_back(t);
    }
    const size_t u = (sections - 1) * (degree + 1), v = 0;
    for (size_t k = 0; k < degree; ++k) {
        const size_t a = u + k + 1, b = u + (k + 1) % degree + 1;
        const size_t A = v + k + 1, B = v + (k + 1) % degree + 1;
        tets.push_back({u, a, b, B});
        tets.push_back({u, a, B, A});
        tets.push_back({u, v, A, B});
    }
    for (const auto& t : tets) REQUIRE(tet_volume_above_threshold(input.vertices, t, 0));
    input.input_cells.assign(tets.size(), 0);
    input.offset_tet_tags.assign(tets.size(), 1);
    rebuild_fixture(input, tets);
    prepare_rod_mesh(input, options());
    REQUIRE(input.rod_state->report["endpoints"] == 0);
    REQUIRE(input.rod_state->report["joints"] == 0);
    REQUIRE(input.rod_state->report["rods"] == 1);
    REQUIRE(input.rod_state->vertex_frame.size() == sections);
    REQUIRE(input.rod_state->protected_input.empty());
    REQUIRE(input.rod_state->frozen_offset.empty());
    REQUIRE(input.junction_protection->centers.empty());
    const auto& path = input.rod_state->report["rod_paths"][0];
    REQUIRE(path["closed"] == true);
    REQUIRE(path["periodic_shared_frame"] == true);
    REQUIRE(path["frame_sequence"].size() == sections + 1);
    REQUIRE(path["frame_sequence"].front() == path["frame_sequence"].back());
    REQUIRE(path["closing_interval_candidate_columns"] == degree);
    auto hybrid = build_prism_dominant_mesh(input, 0);
    REQUIRE(hybrid.report["prisms"] == sections * degree);
    REQUIRE(hybrid.report["band_tetrahedra"] == 0);
    REQUIRE_NOTHROW(validate_prism_dominant_mesh(input, hybrid, 0));
}
