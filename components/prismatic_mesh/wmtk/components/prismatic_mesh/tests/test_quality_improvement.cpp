#include <mshio/mshio.h>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <wmtk/components/prismatic_mesh/quality/bulk.hpp>
#include <wmtk/components/prismatic_mesh/quality/prism_region.hpp>
#include <wmtk/components/prismatic_mesh/quality/region_state.hpp>
#include <wmtk/components/prismatic_mesh/quality_improvement.hpp>
namespace t = wmtk::components::prismatic_mesh::quality;
namespace native = t::native;
using Tet = t::Tet;
namespace {
void require(bool ok, const char* text)
{
    INFO(text);
    REQUIRE(ok);
}
t::Mesh mesh(
    std::vector<t::V> points,
    std::vector<t::Cell> cc,
    const std::vector<size_t>& input,
    const std::map<size_t, size_t>& corr = {})
{
    t::Mesh m;
    m.p = points;
    m.initial = m.base = points;
    m.original_n = points.size();
    m.cells = cc;
    m.input.assign(points.size(), false);
    m.outer = m.input;
    m.thickness.assign(points.size(), NAN);
    m.radius.assign(points.size(), 0);
    m.sizing.assign(points.size(), 1);
    m.correspondence.assign(points.size(), -1);
    for (size_t v : input) {
        m.input[v] = true;
        m.thickness[v] = 0;
    }
    for (auto [a, b] : corr) {
        m.correspondence[a] = b;
        m.base[a] = points[b];
        m.thickness[a] = (points[a] - points[b]).norm();
    }
    m.lo = m.hi = points[0];
    for (const auto& p : points) {
        m.lo = m.lo.cwiseMin(p);
        m.hi = m.hi.cwiseMax(p);
    }
    for (auto& c : m.cells) {
        if (c.type == 4 && native::determinant(m.p, c.v) < 0) std::swap(c.v[0], c.v[1]);
        require(std::isfinite(native::quality(m.p, c)), "selftest_nonpositive_source");
    }
    m.rebuild();
    return m;
}
} // namespace
TEST_CASE(
    "region quality operations preserve geometry and exact positive cells",
    "[prismatic_quality]")
{
    {
        const double next = std::nextafter(1., 2.);
        std::vector<t::V> p{{1, 0, 0}, {next, 0, 0}, {1, 1, 0}, {1, 0, 1}, {1, -1, -1}};
        std::vector<Tet> tt{{0, 1, 2, 3}, {0, 1, 3, 4}, {0, 1, 4, 2}};
        wmtk::OptimizerParameters parameters;
        parameters.l = 1;
        parameters.diag_l = 3;
        t::Bulk backend(
            parameters,
            p,
            tt,
            std::vector<uint8_t>(5, true),
            std::vector<double>(5, 1),
            {});
        backend.set_budget(10, [] { return false; });
        bool found = false;
        for (const auto& edge : backend.get_edges()) {
            size_t a = edge.vid(backend), b = edge.switch_vertex(backend).vid(backend);
            if (std::min(a, b) == 0 && std::max(a, b) == 1) {
                found = true;
                std::vector<t::Bulk::Tuple> children;
                require(
                    !backend.split_edge(edge, children),
                    "unrepresentable_midpoint_split_was_accepted");
                break;
            }
        }
        require(
            found && backend.rejected_unrounded_splits > 0,
            "unsafe_midpoint_fixture_not_exercised");
        require(
            backend.get_tets().size() == 3 && backend.get_vertices().size() == 5,
            "rejected_split_not_rolled_back");
        backend.validate_fixed_boundary();
        require(backend.round_and_check_all_rounded(), "rejected_split_left_unrounded_points");
    }
    {
        auto m = mesh(
            {{0, 0, 0},
             {1, 1, 0},
             {1, 0, 0},
             {0, 1, 0},
             {0, 0, 1},
             {1, 1, 1},
             {1, 0, 1},
             {0, 1, 1},
             {.5, .5, -1},
             {.5, .5, 2}},
            {{6, 2, {0, 2, 1, 4, 6, 5}},
             {6, 2, {0, 1, 3, 4, 5, 7}},
             {4, 2, {8, 0, 2, 1}},
             {4, 3, {8, 0, 1, 3}},
             {4, 3, {9, 4, 5, 6}},
             {4, 3, {9, 4, 7, 5}}},
            {0, 1, 2, 3},
            {{4, 0}, {5, 1}, {6, 2}, {7, 3}});
        t::Rules r(m, t::faces_of(m.cells));
        require(r.patch_area.size() == 1, "cell_type_boundary_mistaken_for_input_geometry_patch");
    }
    {
        // Three genuinely independent input planes can be nearly parallel. A
        // covariance eigenvalue cutoff would incorrectly invent a free DOF.
        auto m = mesh(
            {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 1, -1e-5}},
            {{4, 2, {0, 1, 2, 3}}},
            {0, 1, 2, 3});
        t::Rules r(m, t::faces_of(m.cells));
        t::InputFreedom f(m, r);
        require(f.tangent[0].norm() < 1e-8, "nearly_parallel_input_corner_not_fixed");
    }
    for (int type : {4, 6, 7}) {
        std::vector<t::V> p =
            type == 4 ? std::vector<t::V>{{0, 0, 0}, {1.2, 0, 0}, {.2, 1, 0}, {.1, .2, .8}}
            : type == 6
                ? std::vector<
                      t::V>{{0, 0, 0}, {1.2, 0, 0}, {.2, 1, 0}, {.1, 0, .7}, {1.1, .1, .8}, {.2, .9, .6}}
                : std::vector<t::V>{{0, 0, 0}, {1.1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {.4, .6, .5}};
        std::vector<size_t> vertices(p.size());
        std::iota(vertices.begin(), vertices.end(), 0);
        auto m = mesh(p, {{type, 2, vertices}}, {});
        const auto g = t::cell_gradient(m, m.cells[0]);
        for (size_t v = 0; v < p.size(); ++v)
            for (int j = 0; j < 3; ++j) {
                const double step = 1e-6;
                auto plus = m.cells[0], minus = plus;
                m.p[v][j] += step;
                const double a = native::quality(m.p, plus);
                m.p[v][j] -= 2 * step;
                const double b = native::quality(m.p, minus);
                m.p[v][j] += step;
                require(
                    std::abs((a - b) / (2 * step) - g[v][j]) <
                        1e-5 * std::max(1., std::abs(g[v][j])),
                    "mixed_native_gradient_mismatch");
            }
    }
    {
        auto m = mesh(
            {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, .1}, {1, 0, .1}, {0, 1, .1}},
            {{6, 2, {0, 1, 2, 3, 4, 5}}},
            {0, 1, 2},
            {{3, 0}, {4, 1}, {5, 2}});
        t::MixedReference ref(m, .1, .2, {});
        native::ConformingRefiner split(m);
        const size_t n = m.p.size();
        require(split.split({0, 3}, true, 100, true, false, true), "axial_split_fixture_failed");
        for (size_t v = n; v < m.p.size(); ++v) {
            ref.midpoint(m, v, split.point_parents[v]);
            require(
                std::abs(ref.height[v] - .05) < 1e-12,
                "interior_prism_vertex_given_full_offset_height");
            require(std::abs(ref.height_error(m, v)) < 1e-12, "interpolated_height_not_satisfied");
        }
        split.validate();
    }
    {
        auto m = mesh(
            {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {.5, .5, .001}},
            {{7, 3, {0, 1, 2, 3, 4}}},
            {0, 1, 2, 3});
        t::Rules r(m, t::faces_of(m.cells));
        t::MixedReference ref(m, .1, .2, {});
        t::InputFreedom freedom(m, r);
        const double old = m.cells[0].energy;
        require(
            t::mixed_vertex_step(m, r, ref, freedom, 4, 10, .01),
            "background_pyramid_apex_still_frozen");
        require(m.cells[0].energy < old && m.p[4][2] > .001, "background_pyramid_not_improved");
    }
    {
        std::vector<t::V> p{{.1, .1, 0}, {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
        for (size_t v = 0; v < 5; ++v) p.push_back(p[v] + t::V(0, 0, 1));
        p.push_back({.5, .5, -1});
        p.push_back({.5, .5, 2});
        std::vector<t::Cell> cc;
        for (int j = 1; j <= 4; ++j) {
            int k = j == 4 ? 1 : j + 1;
            cc.push_back({6, 2, {0, size_t(j), size_t(k), 5, size_t(j + 5), size_t(k + 5)}});
            cc.push_back({4, 3, {10, 0, size_t(j), size_t(k)}});
            cc.push_back({4, 3, {11, 5, size_t(j + 5), size_t(k + 5)}});
        }
        auto m = mesh(p, cc, {0, 1, 2, 3, 4}, {{5, 0}, {6, 1}, {7, 2}, {8, 3}, {9, 4}});
        t::Rules r(m, t::faces_of(m.cells));
        const auto topo = t::topology(m, t::faces_of(m.cells));
        const double vol = native::volume(m);
        auto op = t::collapse_prism_column(m, r, 0, 1);
        require(
            op["new_cells"].get<size_t>() < op["old_cells"].get<size_t>(),
            "paired_column_collapse_no_reduction");
        require(t::topology(m, t::faces_of(m.cells)) == topo, "paired_column_collapse_tag_change");
        require(std::abs(native::volume(m) - vol) < 1e-10, "paired_column_collapse_domain_changed");
        r.audit(m, t::faces_of(m.cells));
    }
    {
        auto m = mesh(
            {{0, 0, 0},
             {.5800602499253712, 1.6046576950355842, 0},
             {2, 0, 0},
             {0, 1, 0},
             {0, 0, 1},
             {.5800602499253712, 1.6046576950355842, 1},
             {2, 0, 1},
             {0, 1, 1},
             {0, 0, -1},
             {.5800602499253712, 1.6046576950355842, -1},
             {2, 0, -1},
             {0, 1, -1},
             {.5, .5, 2},
             {.5, .5, -2}},
            {{6, 2, {0, 2, 1, 4, 6, 5}},
             {6, 2, {0, 1, 3, 4, 5, 7}},
             {6, 2, {0, 1, 2, 8, 9, 10}},
             {6, 2, {0, 3, 1, 8, 11, 9}},
             {4, 3, {12, 4, 5, 6}},
             {4, 3, {12, 4, 7, 5}},
             {4, 3, {13, 8, 9, 10}},
             {4, 3, {13, 8, 11, 9}}},
            {0, 1, 2, 3},
            {{4, 0}, {5, 1}, {6, 2}, {7, 3}, {8, 0}, {9, 1}, {10, 2}, {11, 3}});
        t::Rules r(m, t::faces_of(m.cells));
        const auto topo = t::topology(m, t::faces_of(m.cells));
        auto op = t::flip_prism_region(m, r, {0, 1});
        require(
            op["prisms_updated"] == 4 && op["prism_layers"] == 2,
            "two_sided_native_prism_region_not_flipped");
        require(t::topology(m, t::faces_of(m.cells)) == topo, "native_prism_flip_tags_changed");
        r.audit(m, t::faces_of(m.cells));
    }
    {
        auto m = mesh({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {.2, .2, .01}}, {{4, 2, {0, 1, 2, 3}}}, {});
        t::initialize_region_sizing(m, .6, .25, .075);
        t::refine_mixed_sizing(m, 20, .15, .075, 1);
        t::refine_mixed_sizing(m, 20, .15, .075, 1);
        require(
            *std::min_element(m.sizing.begin(), m.sizing.end()) >= .5,
            "sizing_refinement_below_thickness");
    }
}

TEST_CASE("region quality options reject invalid and wrapped budgets", "[prismatic_quality]")
{
    using Options = wmtk::components::prismatic_mesh::QualityImprovementOptions;
    REQUIRE(Options{}.thickness_weight == 100);
    REQUIRE_THROWS(Options::from_json({{"surface_split_budget", -1}}));
    REQUIRE_THROWS(Options::from_json({{"quality_target", 18}, {"smoothing_target", 20}}));
    REQUIRE_THROWS(Options::from_json({{"time_limit", 0}}));
    REQUIRE_THROWS(Options::from_json({{"thickness_weight", -1}}));
    REQUIRE_THROWS(Options::from_json({{"thickness_smoothing_weight", -1}}));
    REQUIRE_THROWS(Options::from_json({{"thickness_tolerance", 0}}));
    REQUIRE_THROWS(Options::from_json({{"thickness_feature_angle", 0}}));
    REQUIRE_THROWS(Options::from_json({{"thickness_feature_angle", 91}}));
    REQUIRE_NOTHROW(Options::from_json({{"cycles", 0}, {"topology_enabled", false}}));
}

namespace {
using namespace wmtk::components::prismatic_mesh;
PrismaticMeshInput construction_fixture(
    const std::vector<wmtk::Vector3d>& extra = {},
    const std::vector<std::array<size_t, 4>>& extra_tets = {},
    const std::vector<int64_t>& extra_parents = {})
{
    PrismaticMeshInput data;
    const size_t n = 6 + extra.size();
    data.vertices.resize(n, 3);
    data.vertices.topRows(6) << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 1;
    data.vertex_tags = {1, 1, 1, 2, 2, 2};
    data.corr_input_vertex = {-1, -1, -1, 0, 1, 2};
    for (size_t i = 0; i < extra.size(); ++i) {
        data.vertices.row(6 + i) = extra[i].transpose();
        const int64_t parent = extra_parents.empty() ? -1 : extra_parents.at(i);
        data.vertex_tags.push_back(parent >= 0 ? 2 : -1);
        data.corr_input_vertex.push_back(parent);
    }
    data.source_vertex_ids.resize(n);
    data.corr_input_vid.resize(n);
    data.input_to_offset_vertices.resize(n);
    for (size_t v = 0; v < n; ++v) {
        data.source_vertex_ids[v] = 100 + v;
        data.corr_input_vid[v] =
            data.corr_input_vertex[v] < 0 ? -1 : 100 + data.corr_input_vertex[v];
        if (data.vertex_tags[v] == 1) data.input_vertices.push_back(v);
        if (data.vertex_tags[v] == 2) {
            data.offset_vertices.push_back(v);
            data.input_to_offset_vertices[data.corr_input_vertex[v]].push_back(v);
        }
    }
    std::vector<std::array<size_t, 4>> tets = {{0, 1, 2, 5}, {0, 1, 5, 4}, {0, 3, 4, 5}};
    tets.insert(tets.end(), extra_tets.begin(), extra_tets.end());
    data.tetrahedra.resize(tets.size(), 4);
    for (size_t i = 0; i < tets.size(); ++i) {
        if (!tet_volume_above_threshold(data.vertices, tets[i], 0))
            std::swap(tets[i][0], tets[i][1]);
        REQUIRE(tet_volume_above_threshold(data.vertices, tets[i], 0));
        for (int j = 0; j < 4; ++j) data.tetrahedra(i, j) = tets[i][j];
    }
    data.input_cells.assign(tets.size(), 0);
    data.offset_tet_tags.assign(tets.size(), 1);
    data.mesh = std::make_unique<wmtk::TetMesh>();
    data.mesh->init_with_isolated_vertices(n, tets);
    REQUIRE(data.mesh->check_mesh_connectivity_validity());
    return data;
}

} // namespace

TEST_CASE(
    "region quality API exports native cells without stale reference IDs",
    "[prismatic_quality]")
{
    using namespace wmtk::components::prismatic_mesh;
    auto input = construction_fixture();
    label_offset_faces(input);
    input.target_thickness = 1;
    const auto before = input.vertices;
    const auto original_tets = input.tetrahedra;
    auto hybrid = build_prism_dominant_mesh(input, 0);
    QualityImprovementOptions options;
    options.cycles = 0;
    auto result = improve_prismatic_mesh_quality(input, hybrid, options);
    REQUIRE(result.cells.size() == 1);
    REQUIRE(result.cells[0].type == HybridCellType::Prism);
    REQUIRE(result.cells[0].amips >= 3);
    auto native_vertices = result.cells[0].vertices;
    std::swap(native_vertices[1], native_vertices[2]);
    std::swap(native_vertices[4], native_vertices[5]);
    std::vector<t::V> points;
    for (Eigen::Index i = 0; i < result.vertices.rows(); ++i)
        points.emplace_back(result.vertices.row(i));
    native::Cell native_cell{6, 2, native_vertices};
    REQUIRE(std::isfinite(native::quality(points, native_cell)));

    REQUIRE(std::isfinite(result.cells[0].amips));
    REQUIRE(input.vertices == before);
    REQUIRE(input.tetrahedra == original_tets);
    wmtk::MeshMap output;
    nlohmann::json report;
    write_improved_prismatic_mesh(result, "mesh.vtu", true, &output, &report);
    REQUIRE(output.size() == 1);
    const auto& data = output.at("mesh.vtu");
    REQUIRE_NOTHROW(data.validate());
    REQUIRE(data.cells.at(wmtk::CellType::Prism).connectivity.rows() == 1);
    const auto& fields = data.cells.at(wmtk::CellType::Prism).attributes;
    REQUIRE(fields.count("AMIPS_native") == 1);
    REQUIRE(fields.count("cell_type") == 1);
    REQUIRE(fields.count("source_tet_ids") == 0);
    REQUIRE(data.vertex_attributes.at("vid").integers()(0, 0) == 100);
    REQUIRE(data.vertex_attributes.at("reference_height_error_ratio").doubles().allFinite());
    REQUIRE(data.vertex_attributes.count("thickness_constrained") == 1);
    REQUIRE(report["quality_improvement"]["configuration"]["thickness_weight"] == 100);
    struct TemporaryExport
    {
        std::filesystem::path path;
        ~TemporaryExport()
        {
            std::error_code error;
            std::filesystem::remove(path, error);
            std::filesystem::remove(
                path.parent_path() / (path.stem().string() + "_hybrid.json"),
                error);
        }
    } file{
        std::filesystem::temp_directory_path() /
        ("wmtk_quality_export_" +
         std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".msh")};
    write_improved_prismatic_mesh(result, file.path);
    const auto msh = mshio::load_msh(file.path.string());
    REQUIRE_NOTHROW(mshio::validate_spec(msh));
    REQUIRE(msh.elements.entity_blocks.size() == 1);
    REQUIRE(msh.elements.entity_blocks.front().element_type == 6);
    const auto& gmsh_vertices = msh.elements.entity_blocks.front().data;
    for (size_t j = 0; j < 6; ++j) REQUIRE(gmsh_vertices[j + 1] == native_vertices[j] + 1);
    options.cycles = 1;
    options.column_collapse_attempts = 0;
    options.surface_split_budget = 1;
    options.surface_flip_attempts = 0;
    auto retried = improve_prismatic_mesh_quality(input, hybrid, options);
    REQUIRE(retried.report["stages"].size() == 4);
    REQUIRE(retried.report["stages"][1].contains("rolled_back"));
    REQUIRE(retried.report["stages"][2]["operations"]["skipped"] == "no_tetrahedra");
    REQUIRE(retried.cells.size() == result.cells.size());
    REQUIRE(retried.vertices == result.vertices);
}

TEST_CASE(
    "region quality is available through the memory component and JSON schema",
    "[prismatic_quality]")
{
    using namespace wmtk::components::prismatic_mesh;
    auto input = construction_fixture();
    wmtk::MeshMap raw;
    write_prismatic_mesh(input, "construction.vtu", nullptr, true, &raw);
    const wmtk::MeshMap meshes{{"construction", raw.at("construction.vtu")}};
    nlohmann::json options = {
        {"application", "prismatic_mesh"},
        {"input", "construction"},
        {"output", "mesh.vtu"},
        {"prism_dominant", true},
        {"iterations", 0},
        {"quality_improvement", true},
        {"export_input_and_shell", true},
        {"quality_improvement_options", {{"cycles", 0}}}};
    const auto result = prismatic_mesh(options, meshes);
    REQUIRE(
        result.second["hybrid"]["quality_improvement"]["configuration"]["thickness_weight"] == 100);
    const auto& configuration = result.second["hybrid"]["quality_improvement"]["configuration"];
    REQUIRE(configuration["thickness_smoothing_weight"] == 10);
    REQUIRE(configuration["thickness_tolerance"] == .05);
    REQUIRE(configuration["thickness_feature_angle"] == 30);
    REQUIRE(result.second["hybrid"]["quality_improvement"].contains("final_energy"));
    REQUIRE(result.second["hybrid"]["quality_improvement"].contains("thickness_target_met"));
    REQUIRE(result.second["hybrid"]["quality_improvement"]["stages"].empty());
    REQUIRE(
        result.first.at("mesh").cells.at(wmtk::CellType::Prism).attributes.count("AMIPS_native") ==
        1);
    REQUIRE(
        result.first.at("input_and_shell").cells.at(wmtk::CellType::Prism).connectivity.rows() ==
        1);
    auto smoothing_options = options;
    smoothing_options["quality_improvement_options"] = {
        {"cycles", 1},
        {"topology_enabled", false},
        {"mixed_smoothing_sweeps", 3}};
    const auto smoothing = prismatic_mesh(smoothing_options, meshes);
    const auto& stages = smoothing.second["hybrid"]["quality_improvement"]["stages"];
    REQUIRE(stages.size() == 2);
    REQUIRE(stages[0]["stage"] == 0);
    REQUIRE(stages[1]["stage"] == 3);
    REQUIRE(
        smoothing.first.at("mesh").cells.at(wmtk::CellType::Prism).connectivity ==
        result.first.at("mesh").cells.at(wmtk::CellType::Prism).connectivity);
    REQUIRE(smoothing.first.at("mesh").V == result.first.at("mesh").V);

    auto construction_options = options;
    construction_options["quality_improvement"] = false;
    const auto construction = prismatic_mesh(construction_options, meshes);
    REQUIRE_FALSE(construction.second["hybrid"].contains("quality_improvement"));
    REQUIRE(
        construction.first.at("mesh")
            .cells.at(wmtk::CellType::Prism)
            .attributes.count("source_tet_ids") == 1);
    construction_options.erase("quality_improvement");
    const auto defaults = prismatic_mesh(construction_options, meshes);
    REQUIRE_FALSE(defaults.second["hybrid"].contains("quality_improvement"));
    REQUIRE(defaults.first.at("mesh").V == construction.first.at("mesh").V);
    options["prism_dominant"] = false;
    REQUIRE_THROWS(prismatic_mesh(options, meshes));
    options["prism_dominant"] = true;
    options["rod_mode"] = true;
    REQUIRE_THROWS(prismatic_mesh(options, meshes));
    options["rod_mode"] = false;
    options["min_tet_volume"] = 1e-8;
    REQUIRE_THROWS(prismatic_mesh(options, meshes));
}

namespace {
t::Mesh thickness_fixture()
{
    return mesh(
        {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, .5}, {1, 0, .5}, {0, 1, .5}},
        {{6, 2, {0, 1, 2, 3, 4, 5}}},
        {0, 1, 2},
        {{3, 0}, {4, 1}, {5, 2}});
}
} // namespace

TEST_CASE("good AMIPS does not stop thickness repair", "[prismatic_quality][prismatic_thickness]")
{
    auto m = thickness_fixture();
    const auto before = m;
    REQUIRE(m.cells[0].energy < 20);
    t::Rules rules(m, t::faces_of(m.cells));
    t::MixedReference reference(m, .25, .5, {});
    t::Deadline deadline(10);
    const auto result = t::smooth_mixed(m, rules, reference, 32, 18, 20, 100, deadline);
    REQUIRE(result["accepted"].get<size_t>() > 0);
    REQUIRE(result["history"].size() > 1);
    REQUIRE(result["initial_thickness"]["points_over_tolerance"] == 3);
    REQUIRE(result["final_thickness"]["target_met"] == true);
    REQUIRE(result["final_thickness"]["maximum_absolute_height_error"].get<double>() <= .05);
    REQUIRE(m.cells[0].energy < 20);
    for (size_t v = 0; v < 3; ++v) REQUIRE(m.p[v] == before.p[v]);
    REQUIRE(std::isfinite(native::quality(m.p, m.cells[0])));

    auto disabled = before;
    t::MixedReference disabled_reference(disabled, .25, .5, {});
    t::Deadline disabled_deadline(10);
    const auto skipped =
        t::smooth_mixed(disabled, rules, disabled_reference, 32, 18, 20, 0, disabled_deadline, 0);
    REQUIRE(skipped["accepted"] == 0);
    REQUIRE(skipped["final_thickness"]["target_met"] == true);
    REQUIRE(disabled.p == before.p);
}

TEST_CASE(
    "height gradients include moved anchors and coupled columns",
    "[prismatic_quality][prismatic_thickness]")
{
    auto m = thickness_fixture();
    t::MixedReference reference(m, .25, .5, {});
    m.p[3][2] = .6;
    m.p[4][2] = .45;
    m.p[5][2] = .55;
    t::ThicknessEnergy energy(m, reference, 100);
    REQUIRE(energy.pairs.size() == 3);
    for (const std::vector<size_t> group :
         {std::vector<size_t>{3}, {0}, {0, 3}, {3, 4}, {0, 3, 4}}) {
        const auto original = m.p;
        t::V gradient = t::V::Zero();
        energy.local(m, group, &gradient);
        for (int d = 0; d < 3; ++d) {
            const double step = 1e-6;
            for (size_t v : group) m.p[v][d] = original[v][d] + step;
            const double plus = energy.local(m, group);
            for (size_t v : group) m.p[v][d] = original[v][d] - step;
            const double minus = energy.local(m, group);
            m.p = original;
            const double numerical = (plus - minus) / (2 * step);
            REQUIRE(
                std::abs(numerical - gradient[d]) <
                1e-5 * std::max({1., std::abs(numerical), std::abs(gradient[d])}));
        }
    }
}

TEST_CASE(
    "height neighbors keep sheets features and reference levels separate",
    "[prismatic_quality][prismatic_thickness]")
{
    auto m = thickness_fixture();
    t::MixedReference reference(m, .25, .5, {});
    reference.normal[5] = t::V::UnitX();
    t::ThicknessEnergy crease(m, reference, 100);
    REQUIRE(crease.pairs.size() == 1);
    REQUIRE(crease.feature_edges == 2);
    reference.normal[4] = -t::V::UnitZ();
    t::ThicknessEnergy opposite(m, reference, 100);
    REQUIRE(opposite.pairs.empty());

    reference.normal[4] = reference.normal[5] = t::V::UnitZ();
    reference.height[4] = .125;
    t::ThicknessEnergy transition(m, reference, 100);
    REQUIRE(transition.pairs.size() == 1);
    REQUIRE(transition.feature_edges == 2);

    const size_t orphan = t::append_volume_vertex(m, {100, 100, 100});
    m.thickness[orphan] = .25;
    reference.extend(m);
    reference.normal[orphan] = t::V::UnitZ();
    reference.height[orphan] = .25;
    m.rebuild();
    t::ThicknessEnergy rebuilt(m, reference, 100);
    REQUIRE(rebuilt.referenced.size() == m.p.size());
    REQUIRE_FALSE(rebuilt.referenced[orphan]);
    REQUIRE(rebuilt.score(m).report["referenced_vertices"] == 3);
}

TEST_CASE(
    "missing interface height references stay fixed during mixed smoothing",
    "[prismatic_quality][prismatic_thickness]")
{
    auto m = thickness_fixture();
    for (size_t v = 3; v < 6; ++v) {
        m.correspondence[v] = -1;
        m.thickness[v] = NAN;
    }
    const auto original = m.p;
    t::Rules rules(m, t::faces_of(m.cells));
    t::MixedReference reference(m, .25, .5, {});
    t::Deadline deadline(10);
    const auto result = t::smooth_mixed(m, rules, reference, 4, 3, 3.1, 100, deadline);
    REQUIRE(result["initial_thickness"]["unreferenced_interface_vertices_fixed"] == 3);
    for (size_t v = 3; v < 6; ++v) REQUIRE(m.p[v] == original[v]);
}

TEST_CASE(
    "an unchanged extreme tet cannot hide checkpoint thickness improvement",
    "[prismatic_quality][prismatic_thickness]")
{
    auto m = mesh(
        {{0, 0, 0},
         {1, 0, 0},
         {0, 1, 0},
         {0, 0, .5},
         {1, 0, .5},
         {0, 1, .5},
         {3, 0, 0},
         {4, 0, 0},
         {3, 1, 0},
         {3, 0, 1e-30}},
        {{6, 2, {0, 1, 2, 3, 4, 5}}, {4, 3, {6, 7, 8, 9}}},
        {0, 1, 2},
        {{3, 0}, {4, 1}, {5, 2}});
    t::MixedReference reference(m, .25, .5, {});
    REQUIRE(m.cells[1].energy > 1e18);
    const auto before = t::mixed_energy_score(m, reference, 18, 100, 10, .05, 30);
    const auto maximum = t::mixed_score(m, 20)["maximum_AMIPS"];
    for (size_t v = 3; v < 6; ++v) m.p[v][2] = .25;
    REQUIRE(std::isfinite(native::quality(m.p, m.cells[0])));
    const auto after = t::mixed_energy_score(m, reference, 18, 100, 10, .05, 30);
    REQUIRE(t::mixed_score(m, 20)["maximum_AMIPS"] == maximum);
    REQUIRE(before.report["total"] == after.report["total"]);
    REQUIRE(after.value < before.value);
    REQUIRE(after.thickness.report["point_energy"] == 0);
    REQUIRE(before.thickness.report["point_energy"].get<double>() > 0);
}
