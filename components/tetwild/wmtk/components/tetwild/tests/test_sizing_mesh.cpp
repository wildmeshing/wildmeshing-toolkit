// The absolute eps and length, the collapse length limit and the sizing mesh.
#include <wmtk/components/tetwild/tetwild.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

// Cases too slow for the Debug CI jobs, Windows Debug above all: Release builds only.
#ifdef NDEBUG
static const std::string release_only = "";
#else
static const std::string release_only = "[.]";
#endif

using namespace wmtk;
using Catch::Matchers::ContainsSubstring;

namespace {

/// The boundary of [0, 1]^3, each side an n by n grid of squares cut in two, facing out.
MeshData grid_cube(int n)
{
    std::map<std::array<int, 3>, int64_t> index;
    std::vector<Eigen::RowVector3d> V;
    std::vector<std::array<int64_t, 3>> F;
    for (int axis = 0; axis < 3; ++axis) {
        // (u, w, axis) is right-handed: counterclockwise in (u, w) faces +axis.
        const int u = (axis + 1) % 3, w = (axis + 2) % 3;
        for (int side = 0; side < 2; ++side) {
            const auto at = [&](int a, int b) {
                std::array<int, 3> g;
                g[axis] = side * n;
                g[u] = a;
                g[w] = b;
                const auto [it, added] = index.emplace(g, int64_t(V.size()));
                if (added) V.emplace_back(double(g[0]) / n, double(g[1]) / n, double(g[2]) / n);
                return it->second;
            };
            for (int i = 0; i < n; ++i) {
                for (int j = 0; j < n; ++j) {
                    const int64_t p = at(i, j), q = at(i + 1, j), r = at(i + 1, j + 1),
                                  s = at(i, j + 1);
                    if (side == 1) {
                        F.push_back({{p, q, r}});
                        F.push_back({{p, r, s}});
                    } else {
                        F.push_back({{p, r, q}});
                        F.push_back({{p, s, r}});
                    }
                }
            }
        }
    }
    MeshData m;
    m.V.resize(Eigen::Index(V.size()), 3);
    for (size_t i = 0; i < V.size(); ++i) m.V.row(Eigen::Index(i)) = V[i];
    auto& T = m.cells[CellType::Triangle].connectivity;
    T.resize(Eigen::Index(F.size()), 3);
    for (size_t i = 0; i < F.size(); ++i) {
        for (int k = 0; k < 3; ++k) T(Eigen::Index(i), k) = F[i][k];
    }
    return m;
}

/// Slabs of the box [x_0, x_n] x [lo, hi]^2, between consecutive planes x = x_i, each as six tets
/// around its diagonal; its field `target_length` is l_i on the plane x_i, linear in x in between.
/// `planes` holds the pairs (x_i, l_i).
MeshData sizing_slabs(const std::vector<std::array<double, 2>>& planes, double lo, double hi)
{
    const int n = int(planes.size()) - 1;
    MeshData m;
    m.V.resize(8 * n, 3);
    DataMatrix<double> length(8 * n, 1);
    auto& T = m.cells[CellType::Tetrahedron].connectivity;
    T.resize(6 * n, 4);
    // From corner 0 to corner 7 one axis at a time, in each of the six orders.
    const int axes[6][2] = {{1, 2}, {1, 4}, {2, 1}, {2, 4}, {4, 1}, {4, 2}};
    for (int s = 0; s < n; ++s) {
        const int v = 8 * s;
        for (int i = 0; i < 8; ++i) {
            const auto& plane = planes[size_t(s + (i & 1))];
            m.V.row(v + i) << plane[0], (i & 2 ? hi : lo), (i & 4 ? hi : lo);
            length(v + i, 0) = plane[1];
        }
        for (int t = 0; t < 6; ++t) {
            T.row(6 * s + t) << v, v + axes[t][0], v + (axes[t][0] | axes[t][1]), v + 7;
        }
    }
    m.vertex_attributes[SizingMesh::field_name].values = length;
    return m;
}

/// The edges of the output's tracked surface (its triangle block), each once: its ends.
std::vector<std::array<Eigen::RowVector3d, 2>> tracked_edges(const MeshData& mesh)
{
    const auto& F = mesh.cell(CellType::Triangle).connectivity;
    std::set<std::array<int64_t, 2>> edges;
    for (Eigen::Index f = 0; f < F.rows(); ++f) {
        for (int k = 0; k < 3; ++k) {
            const int64_t a = F(f, k), b = F(f, (k + 1) % 3);
            edges.insert({{std::min(a, b), std::max(a, b)}});
        }
    }
    std::vector<std::array<Eigen::RowVector3d, 2>> out;
    for (const auto& e : edges) out.push_back({{mesh.V.row(e[0]), mesh.V.row(e[1])}});
    return out;
}

double longest_tracked_edge(const MeshData& mesh)
{
    double m = 0;
    for (const auto& e : tracked_edges(mesh)) m = std::max(m, (e[0] - e[1]).norm());
    return m;
}

} // namespace

TEST_CASE("tetwild-absolute-eps-and-length", "[tetwild][sizing]" + release_only)
{
    // The same run, by relative or by absolute values: the bounding box diagonal is sqrt(3).
    const MeshMap meshes{{"cube", grid_cube(1)}};
    const double diag = std::sqrt(3.0);
    const nlohmann::json options = {
        {"input", {"cube"}},
        {"max_iterations", 1},
        {"stop_energy", 10},
        {"num_threads", 0}};
    nlohmann::json relative = options, absolute = options;
    relative.update({{"eps_rel", 2e-3}, {"length_rel", 0.1}});
    absolute.update({{"eps", 2e-3 * diag}, {"length", 0.1 * diag}});
    const auto r = components::tetwild::tetwild(relative, meshes);
    const auto a = components::tetwild::tetwild(absolute, meshes);
    CHECK(r.second["eps"].get<double>() == a.second["eps"].get<double>());
    CHECK(r.second["length"].get<double>() == a.second["length"].get<double>());
    CHECK(a.second["length"].get<double>() == 0.1 * diag);
    CHECK_FALSE(a.second["sizing_mesh"].get<bool>());
    CHECK(r.first.at("mesh").V == a.first.at("mesh").V);
}

TEST_CASE("tetwild-limit-collapse-length", "[tetwild][sizing]" + release_only)
{
    // The background mesh's edges are about 0.09 long, under 0.8 of the target, so the cube's
    // sides are collapsed, and without the limit a collapse leaves tracked edges longer than the
    // split length, which the run ends on (1.55 l when measured). With it they stay near 4/3 of
    // the target (smoothing may still stretch them a little: 1.31 l).
    const MeshMap meshes{{"cube", grid_cube(1)}};
    const double l = 0.12;
    double longest[2] = {0, 0};
    for (const bool limit : {false, true}) {
        const auto r = components::tetwild::tetwild(
            {{"input", {"cube"}},
             {"length", l},
             {"stop_energy", 10},
             {"max_iterations", 1},
             {"limit_collapse_length", limit},
             {"num_threads", 0}},
            meshes);
        longest[limit] = longest_tracked_edge(r.first.at("mesh"));
    }
    INFO(
        "longest tracked edge without the limit " << longest[0] / l << " l, with it "
                                                  << longest[1] / l << " l");
    CHECK(longest[0] > 4. / 3. * l * 1.1);
    CHECK(longest[1] <= 4. / 3. * l * 1.05);
}

TEST_CASE("tetwild-sizing-mesh", "[tetwild][sizing]" + release_only)
{
    // A background mesh over the whole box whose length is 0.06 up to the cube's side x = 0 and
    // grows to the global 0.2 at x = 0.3: the tracked edges on that side are split to the field's
    // length, those on the side x = 1 keep the background mesh's (about 0.09).
    const MeshMap meshes{
        {"cube", grid_cube(1)},
        {"sizing",
         sizing_slabs({{{-0.2, 0.06}}, {{0, 0.06}}, {{0.3, 0.2}}, {{1.2, 0.2}}}, -0.2, 1.2)}};
    const auto r = components::tetwild::tetwild(
        {{"input", {"cube"}},
         {"length", 0.2},
         {"sizing_mesh", "sizing"},
         {"stop_energy", 10},
         {"max_iterations", 1},
         {"num_threads", 0}},
        meshes);
    CHECK(r.second["sizing_mesh"].get<bool>());
    double sum[2] = {0, 0};
    int count[2] = {0, 0};
    for (const auto& e : tracked_edges(r.first.at("mesh"))) {
        for (const int side : {0, 1}) {
            if (std::abs(e[0].x() - side) < 0.01 && std::abs(e[1].x() - side) < 0.01) {
                sum[side] += (e[0] - e[1]).norm();
                ++count[side];
            }
        }
    }
    REQUIRE(count[0] > 0);
    REQUIRE(count[1] > 0);
    const double mean0 = sum[0] / count[0], mean1 = sum[1] / count[1];
    INFO(
        "mean tracked edge on the side x = 0 " << mean0 << " (" << count[0] << " edges), on x = 1 "
                                               << mean1 << " (" << count[1] << ")");
    CHECK(mean0 < 4. / 3. * 0.06);
    CHECK(mean0 < 0.7 * mean1);
}

TEST_CASE("tetwild-sizing-mesh-errors", "[tetwild][sizing]")
{
    // A bad sizing mesh fails before any work, and so do the options with the legacy optimizer,
    // which would ignore them.
    const MeshData sizing = sizing_slabs({{{-1, 0.1}}, {{2, 0.1}}}, -1, 2);
    MeshMap meshes{{"cube", grid_cube(1)}, {"sizing", sizing}};
    const auto run = [&meshes](const nlohmann::json& extra) {
        nlohmann::json j = {{"input", {"cube"}}, {"num_threads", 0}};
        j.update(extra);
        return components::tetwild::tetwild(j, meshes);
    };
    CHECK_THROWS_WITH(run({{"sizing_mesh", "missing"}}), ContainsSubstring("missing"));
    CHECK_THROWS_WITH(run({{"sizing_mesh", "cube"}}), ContainsSubstring("target_length"));
    // A surface with the field: a sizing mesh of tetwild's has tets.
    meshes["surface"] = meshes["cube"];
    meshes["surface"].vertex_attributes[SizingMesh::field_name].values =
        DataMatrix<double>(DataMatrix<double>::Constant(meshes["cube"].V.rows(), 1, 0.1));
    CHECK_THROWS_WITH(run({{"sizing_mesh", "surface"}}), ContainsSubstring("tetrahedra"));
    DataMatrix<double> bad = sizing.vertex_attributes.at(SizingMesh::field_name).doubles();
    bad(1, 0) = -1;
    meshes["sizing"].vertex_attributes[SizingMesh::field_name].values = bad;
    CHECK_THROWS_WITH(run({{"sizing_mesh", "sizing"}}), ContainsSubstring("positive"));
    meshes["sizing"] = sizing;
    CHECK_THROWS_WITH(
        run({{"sizing_mesh", "sizing"}, {"use_legacy_code", true}}),
        ContainsSubstring("sizing_mesh: not with use_legacy_code"));
    CHECK_THROWS_WITH(
        run({{"limit_collapse_length", true}, {"use_legacy_code", true}}),
        ContainsSubstring("limit_collapse_length: not with use_legacy_code"));
}

TEST_CASE("tetwild-sizing-mesh-file-route", "[tetwild][sizing]" + release_only)
{
    // The sizing mesh as a file, relative to input_dir: the same mesh as on the memory route. Its
    // 0.2 everywhere, against the default length of 0.087, leaves a much coarser mesh than none.
    namespace fs = std::filesystem;
    const fs::path dir = fs::current_path() / "tetwild_sizing_mesh_file_route";
    fs::remove_all(dir);
    fs::create_directories(dir);
    const MeshData cube = grid_cube(1);
    const MeshData sizing = sizing_slabs({{{-1, 0.2}}, {{2, 0.2}}}, -1, 2);
    {
        std::ofstream o(dir / "cube.obj");
        o.precision(17);
        for (Eigen::Index v = 0; v < cube.V.rows(); ++v)
            o << "v " << cube.V(v, 0) << " " << cube.V(v, 1) << " " << cube.V(v, 2) << "\n";
        const auto& F = cube.cell(CellType::Triangle).connectivity;
        for (Eigen::Index f = 0; f < F.rows(); ++f)
            o << "f " << F(f, 0) + 1 << " " << F(f, 1) + 1 << " " << F(f, 2) + 1 << "\n";
    }
    io::write_mesh(dir / "sizing.msh", sizing);

    const nlohmann::json options = {{"stop_energy", 10}, {"max_iterations", 1}, {"num_threads", 0}};
    nlohmann::json file = options;
    file.update({
        {"application", "tetwild"},
        {"input", {"cube.obj"}},
        {"input_dir", dir.string()},
        {"sizing_mesh", "sizing.msh"},
        {"output", (dir / "out").string()},
        {"report", (dir / "report.json").string()},
        {"write_vtu", false},
    });
    components::tetwild::tetwild(file);
    nlohmann::json memory = options;
    memory.update({{"input", {"cube"}}, {"sizing_mesh", "sizing"}});
    const auto r =
        components::tetwild::tetwild(memory, MeshMap{{"cube", cube}, {"sizing", sizing}});
    CHECK(io::read_mesh(dir / "out_final.msh").V == r.first.at("mesh").V);
    CHECK(nlohmann::json::parse(std::ifstream(dir / "report.json"))["sizing_mesh"].get<bool>());
    fs::remove_all(dir);
}
