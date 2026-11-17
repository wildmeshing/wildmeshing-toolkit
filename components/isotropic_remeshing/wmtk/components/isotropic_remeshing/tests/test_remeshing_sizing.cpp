// The absolute eps and length, and the sizing mesh.
#include <wmtk/components/isotropic_remeshing/IsotropicRemeshing.h>
#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/utils/SizingMesh.hpp>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

using namespace wmtk;
using namespace components::isotropic_remeshing;

namespace {

/// An n x n grid over [0, 1]^2, two counter-clockwise triangles a square, lifted by a bump of
/// height `h` in the middle.
MeshData bumpy_grid(int n, double h)
{
    MeshData m;
    m.V.resize((n + 1) * (n + 1), 3);
    for (int i = 0; i <= n; ++i) {
        for (int j = 0; j <= n; ++j) {
            const double x = double(i) / n, y = double(j) / n;
            m.V.row(i * (n + 1) + j) << x, y, h * std::sin(M_PI * x) * std::sin(M_PI * y);
        }
    }
    auto& F = m.cells[CellType::Triangle].connectivity;
    F.resize(2 * n * n, 3);
    const auto id = [n](int i, int j) { return int64_t(i * (n + 1) + j); };
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            F.row(2 * (i * n + j)) << id(i, j), id(i + 1, j), id(i + 1, j + 1);
            F.row(2 * (i * n + j) + 1) << id(i, j), id(i + 1, j + 1), id(i, j + 1);
        }
    }
    return m;
}

/// The unit square in z = 0 as two triangles, its field target_length `left` on its left side
/// and `right` on its right.
MeshData sizing_square(double left, double right)
{
    MeshData sizing;
    sizing.V.resize(4, 3);
    sizing.V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0;
    sizing.cells[CellType::Triangle].connectivity.resize(2, 3);
    sizing.cells[CellType::Triangle].connectivity << 0, 1, 2, 0, 2, 3;
    DataMatrix<double> length(4, 1);
    length << left, right, right, left;
    sizing.vertex_attributes[SizingMesh::field_name].values = length;
    return sizing;
}

/// The mean length of the edges of `mesh` whose midpoint is left of x = 0.25, and of those
/// right of x = 0.75.
std::array<double, 2> strip_means(const MeshData& mesh)
{
    const auto& F = mesh.cell(CellType::Triangle).connectivity;
    std::set<std::array<int64_t, 2>> edges;
    for (Eigen::Index f = 0; f < F.rows(); ++f) {
        for (int k = 0; k < 3; ++k) {
            const int64_t a = F(f, k), b = F(f, (k + 1) % 3);
            edges.insert({{std::min(a, b), std::max(a, b)}});
        }
    }
    double sum[2] = {0, 0};
    int count[2] = {0, 0};
    for (const auto& [a, b] : edges) {
        const double x = 0.5 * (mesh.V(a, 0) + mesh.V(b, 0));
        if (x > 0.25 && x < 0.75) continue;
        sum[x > 0.5] += (mesh.V.row(a) - mesh.V.row(b)).norm();
        ++count[x > 0.5];
    }
    REQUIRE(count[0] > 0);
    REQUIRE(count[1] > 0);
    return {{sum[0] / count[0], sum[1] / count[1]}};
}

struct TempDir
{
    std::filesystem::path path;
    TempDir()
        : path(
              std::filesystem::temp_directory_path() /
              ("wmtk_remeshing_sizing_" + std::to_string(std::random_device{}())))
    {
        std::filesystem::create_directories(path);
    }
    ~TempDir() { std::filesystem::remove_all(path); }
};

} // namespace

TEST_CASE("sizing-absolute-eps-and-length", "[test_remeshing][sizing]")
{
    // The same run by relative or by absolute values, the diagonal computed as the remeshing
    // computes it: the same mesh.
    const MeshData grid = bumpy_grid(8, 0.2);
    const MeshMap meshes{{"grid", grid}};
    const Eigen::Vector3d lo = grid.V.colwise().minCoeff(), hi = grid.V.colwise().maxCoeff();
    const double diag = (lo - hi).norm();
    const auto relative = isotropic_remeshing(
        {{"input", "grid"}, {"eps_rel", 0.01}, {"length_rel", 0.08}, {"max_iterations", 2}},
        meshes);
    const auto absolute = isotropic_remeshing(
        {{"input", "grid"}, {"eps", 0.01 * diag}, {"length", 0.08 * diag}, {"max_iterations", 2}},
        meshes);
    // A positive absolute value wins over the relative one.
    const auto both = isotropic_remeshing(
        {{"input", "grid"},
         {"eps", 0.01 * diag},
         {"eps_rel", 0.5},
         {"length", 0.08 * diag},
         {"length_rel", 0.5},
         {"max_iterations", 2}},
        meshes);
    CHECK(relative.second["eps"].get<double>() == 0.01 * diag);
    CHECK(relative.second["target_length"].get<double>() == 0.08 * diag);
    for (const auto* r : {&absolute, &both}) {
        CHECK(r->second["eps"] == relative.second["eps"]);
        CHECK(r->second["target_length"] == relative.second["target_length"]);
        CHECK(r->first.at("mesh").V == relative.first.at("mesh").V);
        CHECK(
            r->first.at("mesh").cell(CellType::Triangle).connectivity ==
            relative.first.at("mesh").cell(CellType::Triangle).connectivity);
    }
    // No envelope: eps 0 in the report.
    CHECK(
        isotropic_remeshing({{"input", "grid"}, {"max_iterations", 1}}, meshes)
            .second["eps"]
            .get<double>() == 0);
}

TEST_CASE("sizing-length-and-length-abs", "[test_remeshing][sizing]")
{
    // length is length_abs under the other components' name: the same run either way, or with
    // both when they agree. Two positive values that differ are refused.
    const MeshMap meshes{{"grid", bumpy_grid(8, 0.2)}};
    const nlohmann::json base = {{"input", "grid"}, {"max_iterations", 2}};
    const auto run = [&](const nlohmann::json& extra) {
        nlohmann::json j = base;
        j.update(extra);
        return isotropic_remeshing(j, meshes);
    };
    const auto old_name = run({{"length_abs", 0.1}});
    for (const auto& extra :
         {nlohmann::json{{"length", 0.1}},
          nlohmann::json{{"length", 0.1}, {"length_abs", 0.1}},
          nlohmann::json{{"length", 0.1}, {"length_rel", 0.5}}}) {
        INFO(extra.dump());
        const auto r = run(extra);
        CHECK(r.second["target_length"].get<double>() == 0.1);
        CHECK(r.first.at("mesh").V == old_name.first.at("mesh").V);
        CHECK(
            r.first.at("mesh").cell(CellType::Triangle).connectivity ==
            old_name.first.at("mesh").cell(CellType::Triangle).connectivity);
    }
    CHECK(old_name.second["target_length"].get<double>() == 0.1);
    CHECK_THROWS(run({{"length", 0.1}, {"length_abs", 0.2}}));
}

TEST_CASE("sizing-mesh-gradient", "[test_remeshing][sizing]")
{
    // A sizing mesh under the unit square whose length grows from 0.02 on the left to 0.12 on
    // the right: the edges on the left are much shorter than those on the right (0.029 and 0.105
    // on average, measured, in 1014 triangles, limit_collapse_length on with the sizing mesh).
    // Without it, at the right side's length, they are alike (0.119 and 0.114, in 188).
    const MeshMap meshes{{"grid", bumpy_grid(10, 0)}, {"sizing", sizing_square(0.02, 0.12)}};
    const nlohmann::json base = {{"input", "grid"}, {"length", 0.12}};
    nlohmann::json with = base;
    with["sizing_mesh"] = "sizing";
    const auto r = isotropic_remeshing(with, meshes);
    const auto uniform = isotropic_remeshing(base, meshes);
    CHECK(r.second["sizing_mesh"].get<bool>());
    CHECK(!uniform.second["sizing_mesh"].get<bool>());
    const auto [left, right] = strip_means(r.first.at("mesh"));
    const auto [uniform_left, uniform_right] = strip_means(uniform.first.at("mesh"));
    INFO(
        "mean edge on the left " << left << ", on the right " << right << "; without the sizing "
                                 << "mesh " << uniform_left << " and " << uniform_right << "; "
                                 << r.second["#f"] << " and " << uniform.second["#f"]
                                 << " triangles");
    CHECK(left < 0.04);
    CHECK(right > 0.08);
    CHECK(left < 0.4 * right);
    CHECK(std::abs(uniform_left - uniform_right) < 0.2 * uniform_right);
    CHECK(r.second["#f"].get<size_t>() > 2 * uniform.second["#f"].get<size_t>());

    // With threads: the same, up to the order of the operations.
    nlohmann::json threaded = with;
    threaded["num_threads"] = 4;
    const auto [threaded_left, threaded_right] =
        strip_means(isotropic_remeshing(threaded, meshes).first.at("mesh"));
    INFO("with 4 threads: " << threaded_left << " and " << threaded_right);
    CHECK(std::abs(threaded_left - left) < 0.1 * left);
    CHECK(std::abs(threaded_right - right) < 0.1 * right);

    // The input as its own sizing mesh, the same field on its vertices: it reads the field, and
    // the remeshing drops it from the input.
    MeshData grid = bumpy_grid(10, 0);
    grid.vertex_attributes[SizingMesh::field_name].values =
        DataMatrix<double>((0.02 + 0.1 * grid.V.col(0).array()).matrix());
    with["sizing_mesh"] = "grid";
    const auto itself = isotropic_remeshing(with, MeshMap{{"grid", grid}});
    const auto [itself_left, itself_right] = strip_means(itself.first.at("mesh"));
    INFO("the input as the sizing mesh: " << itself_left << " and " << itself_right);
    CHECK(std::abs(itself_left - left) < 0.1 * left);
    CHECK(std::abs(itself_right - right) < 0.1 * right);
}

TEST_CASE("sizing-mesh-on-a-feature-curve", "[test_remeshing][sizing][features]")
{
    // A feature curve across the gradient, along y = 0.5: its edges follow the field as the
    // surface's do, short on the left and long on the right (0.039 and 0.101 measured).
    const int n = 10;
    MeshData curve = bumpy_grid(n, 0);
    curve.cells.clear();
    auto& E = curve.cells[CellType::Edge].connectivity;
    E.resize(n, 2);
    for (int i = 0; i < n; ++i) E.row(i) << i * (n + 1) + n / 2, (i + 1) * (n + 1) + n / 2;
    const auto r = isotropic_remeshing(
        {{"input", "grid"},
         {"features", "curve"},
         {"length", 0.12},
         {"sizing_mesh", "sizing"},
         {"swap_quality", true}},
        MeshMap{
            {"grid", bumpy_grid(n, 0)},
            {"curve", curve},
            {"sizing", sizing_square(0.02, 0.12)}});
    const MeshData& edges = r.first.at("edges");
    const auto& C = edges.cell(CellType::Edge).connectivity;
    double sum[2] = {0, 0};
    int count[2] = {0, 0};
    for (Eigen::Index i = 0; i < C.rows(); ++i) {
        const Eigen::RowVector3d a = edges.V.row(C(i, 0)), b = edges.V.row(C(i, 1));
        CHECK(std::abs(a.y() - 0.5) < 1e-12); // the curve stays where it is
        const double x = 0.5 * (a.x() + b.x());
        if (x > 0.25 && x < 0.75) continue;
        sum[x > 0.5] += (a - b).norm();
        ++count[x > 0.5];
    }
    REQUIRE(count[0] > 0);
    REQUIRE(count[1] > 0);
    const double left = sum[0] / count[0], right = sum[1] / count[1];
    INFO(
        "mean curve edge on the left " << left << " (" << count[0] << "), on the right " << right
                                       << " (" << count[1] << ")");
    // Splits halve the input's edges of 0.1, so the left side ends between 0.025 and 0.05.
    CHECK(left < 0.05);
    CHECK(right > 0.08);
    CHECK(left < 0.5 * right);
}

TEST_CASE("sizing-mesh-limits-the-collapses-to-the-local-target", "[test_remeshing][sizing]")
{
    // A grid of unit squares under a sizing mesh from 1.5 on the left to 3 on the right: the
    // collapse pass, with limit_collapse_length, leaves no edge longer than 4/3 of its own
    // target, the mean of the field at its ends; without the limit some edge is. With it the
    // longest is at the bound (1.3333 measured, 1.84 without), up to the rounding of the ratio.
    const int n = 10;
    std::vector<Eigen::Vector3d> v;
    std::vector<std::array<size_t, 3>> tris;
    for (int i = 0; i <= n; ++i) {
        for (int j = 0; j <= n; ++j) v.emplace_back(i, j, 0);
    }
    const auto id = [](int i, int j) { return size_t(i * (n + 1) + j); };
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            tris.push_back({{id(i, j), id(i + 1, j), id(i + 1, j + 1)}});
            tris.push_back({{id(i, j), id(i + 1, j + 1), id(i, j + 1)}});
        }
    }
    MeshData field = sizing_square(1.5, 3);
    field.V *= n;
    const auto sizing = std::make_shared<SizingMesh>(field, SizingMesh::Kind::Surface);
    double worst[2] = {0, 0};
    for (const bool limit : {false, true}) {
        IsotropicRemeshing m(v, 0);
        m.create_mesh(v.size(), tris);
        m.set_sizing_mesh(sizing);
        m.set_limit_collapse_length(limit);
        // The global length is far longer: the field alone sets the targets.
        m.collapse_remeshing(10);
        REQUIRE(m.check_mesh_connectivity_validity());
        CHECK(m.get_vertices().size() < v.size());
        for (const auto& e : m.get_edges()) {
            const Eigen::Vector3d& a = m.vertex_attrs[e.vid(m)].pos;
            const Eigen::Vector3d& b = m.vertex_attrs[e.switch_vertex(m).vid(m)].pos;
            const double target = 0.5 * ((*sizing)(a) + (*sizing)(b));
            worst[limit] = std::max(worst[limit], (a - b).norm() / target);
        }
    }
    INFO("longest edge over its target without the limit " << worst[0] << ", with it " << worst[1]);
    CHECK(worst[0] > 4. / 3.);
    CHECK(worst[1] <= 4. / 3. * (1 + 1e-12));
}

TEST_CASE("sizing-mesh-errors", "[test_remeshing][sizing]")
{
    MeshData no_field = sizing_square(0.1, 0.1);
    no_field.vertex_attributes.clear();
    MeshData negative = sizing_square(0.1, 0.1);
    negative.vertex_attributes[SizingMesh::field_name].values =
        DataMatrix<double>(DataMatrix<double>::Constant(4, 1, -1.0));
    MeshData two_columns = sizing_square(0.1, 0.1);
    two_columns.vertex_attributes[SizingMesh::field_name].values =
        DataMatrix<double>(DataMatrix<double>::Constant(4, 2, 0.1));
    MeshData edges_only = sizing_square(0.1, 0.1);
    edges_only.cells.clear();
    edges_only.cells[CellType::Edge].connectivity.resize(1, 2);
    edges_only.cells[CellType::Edge].connectivity << 0, 1;
    const MeshMap meshes{
        {"grid", bumpy_grid(4, 0)},
        {"no_field", no_field},
        {"negative", negative},
        {"two_columns", two_columns},
        {"edges_only", edges_only}};
    for (const std::string name :
         {"no_field", "negative", "two_columns", "edges_only", "missing"}) {
        INFO(name);
        CHECK_THROWS(isotropic_remeshing({{"input", "grid"}, {"sizing_mesh", name}}, meshes));
    }
    // A Planar one has no surface to read in 3D.
    const std::vector<Eigen::Vector3d> v = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    const std::vector<std::array<size_t, 3>> tris = {{{0, 1, 2}}};
    IsotropicRemeshing m(v, 0);
    m.create_mesh(v.size(), tris);
    CHECK_THROWS(m.set_sizing_mesh(
        std::make_shared<SizingMesh>(sizing_square(0.1, 0.1), SizingMesh::Kind::Planar)));
    CHECK(!m.has_sizing_mesh());
}

TEST_CASE("sizing-mesh-file-route", "[test_remeshing][sizing]")
{
    // The sizing mesh as a file, relative to input_dir: the same run as on the memory route.
    const TempDir dir;
    const MeshData grid = bumpy_grid(10, 0);
    io::write_mesh(dir.path / "grid.msh", grid);
    io::write_mesh(dir.path / "sizing.msh", sizing_square(0.02, 0.12));
    const nlohmann::json options = {{"length", 0.12}, {"max_iterations", 2}};
    nlohmann::json file = options;
    file.update({
        {"application", "isotropic_remeshing"},
        {"input", "grid.msh"},
        {"input_dir", dir.path.string()},
        {"sizing_mesh", "sizing.msh"},
        {"output", (dir.path / "out.obj").string()},
        {"report", (dir.path / "report.json").string()},
    });
    isotropic_remeshing(file);
    nlohmann::json memory = options;
    memory.update({{"input", "grid"}, {"sizing_mesh", "sizing"}});
    const auto r =
        isotropic_remeshing(memory, MeshMap{{"grid", grid}, {"sizing", sizing_square(0.02, 0.12)}});
    nlohmann::json report;
    std::ifstream(dir.path / "report.json") >> report;
    CHECK(report["sizing_mesh"].get<bool>());
    CHECK(report["#v"] == r.second["#v"]);
    CHECK(report["#f"] == r.second["#f"]);
    CHECK(report["avg_length"] == r.second["avg_length"]);
    const MeshData out = io::read_mesh(dir.path / "out.obj");
    REQUIRE(out.V.rows() == r.first.at("mesh").V.rows());
    CHECK(out.V == r.first.at("mesh").V); // the .obj holds every digit
    // A missing file fails before any work.
    file["sizing_mesh"] = "none.msh";
    CHECK_THROWS(isotropic_remeshing(file));
}
