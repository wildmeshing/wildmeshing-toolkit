// The collapse length limit and the sizing mesh, in 2D and 3D, alone and with sizing_field's
// regions.
#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/simwild/SimWildMesh.h>
#include <wmtk/components/simwild/SimWildMeshTri.hpp>
#include <wmtk/components/simwild/simwild.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/io/tagged_mesh_data.hpp>
#include <wmtk/utils/SizingMesh.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <vector>

// Cases too slow for the Debug CI jobs, Windows Debug above all: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif

using namespace wmtk;
using components::simwild::simwild;

namespace {

/// The boundary of [0, 1]^2 as one closed polyline, `n` segments per side: region tag_0.
MeshData square_curve(int n)
{
    MeshData m;
    m.V.resize(4 * n, 3);
    const double corners[5][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}};
    for (int side = 0; side < 4; ++side) {
        for (int i = 0; i < n; ++i) {
            const double t = double(i) / n;
            m.V.row(side * n + i) << (1 - t) * corners[side][0] + t * corners[side + 1][0],
                (1 - t) * corners[side][1] + t * corners[side + 1][1], 0;
        }
    }
    auto& E = m.cells[CellType::Edge].connectivity;
    E.resize(4 * n, 2);
    for (int i = 0; i < 4 * n; ++i) E.row(i) << i, (i + 1) % (4 * n);
    return m;
}

/// The unit cube's surface: region tag_0.
MeshData unit_cube()
{
    MatrixXd V(8, 3);
    V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1;
    MatrixXi F(12, 3);
    F << 0, 3, 2, 0, 2, 1, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2, 0, 4, 7, 0, 7, 3,
        1, 2, 6, 1, 6, 5;
    return io::geometry_data(V, F, CellType::Triangle);
}

/// The grid of n[0] x n[1] x n[2] boxes over [lo, hi], each cut into six positively oriented
/// tets (Kuhn's, which match across the boxes). Vertex (i, j, k) is V[(k (n1 + 1) + j) (n0 + 1)
/// + i].
void tet_grid(
    const Vector3d& lo,
    const Vector3d& hi,
    const std::array<int, 3>& n,
    std::vector<Vector3d>& V,
    std::vector<std::array<size_t, 4>>& T)
{
    const auto id = [&](const std::array<int, 3>& c) {
        return size_t((c[2] * (n[1] + 1) + c[1]) * (n[0] + 1) + c[0]);
    };
    for (int k = 0; k <= n[2]; ++k) {
        for (int j = 0; j <= n[1]; ++j) {
            for (int i = 0; i <= n[0]; ++i) {
                const Vector3d t(double(i) / n[0], double(j) / n[1], double(k) / n[2]);
                V.push_back(lo + (hi - lo).cwiseProduct(t));
            }
        }
    }
    const std::array<std::array<int, 3>, 6> paths = {
        {{{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}}, {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}}};
    for (int k = 0; k < n[2]; ++k) {
        for (int j = 0; j < n[1]; ++j) {
            for (int i = 0; i < n[0]; ++i) {
                for (const auto& path : paths) {
                    std::array<int, 3> c = {{i, j, k}};
                    std::array<size_t, 4> t;
                    t[0] = id(c);
                    for (int s = 0; s < 3; ++s) {
                        ++c[path[s]];
                        t[s + 1] = id(c);
                    }
                    const Vector3d &a = V[t[0]], &b = V[t[1]], &d = V[t[2]], &e = V[t[3]];
                    if ((b - a).cross(d - a).dot(e - a) < 0) std::swap(t[2], t[3]);
                    T.push_back(t);
                }
            }
        }
    }
}

/// The unit cube as a tagged tet mesh of n^3 boxes: region a below y = 0.5, b above.
MeshData tagged_cube(int n)
{
    std::vector<Vector3d> V;
    std::vector<std::array<size_t, 4>> T;
    tet_grid(Vector3d(0, 0, 0), Vector3d(1, 1, 1), {{n, n, n}}, V, T);
    MeshData mesh;
    mesh.V.resize(Eigen::Index(V.size()), 3);
    for (size_t v = 0; v < V.size(); ++v) mesh.V.row(Eigen::Index(v)) = V[v].transpose();
    auto& block = mesh.cells[CellType::Tetrahedron];
    block.connectivity.resize(Eigen::Index(T.size()), 4);
    DataMatrix<uint8_t> tags = DataMatrix<uint8_t>::Zero(Eigen::Index(T.size()), 2);
    for (size_t t = 0; t < T.size(); ++t) {
        double y = 0;
        for (int c = 0; c < 4; ++c) {
            block.connectivity(Eigen::Index(t), c) = int64_t(T[t][size_t(c)]);
            y += V[T[t][size_t(c)]].y() / 4;
        }
        tags(Eigen::Index(t), y < 0.5 ? 0 : 1) = 1;
    }
    block.attributes["tags"] = {tags, {"a", "b"}};
    return mesh;
}

/// The rectangle [x0, x1] x [y0, y1] as two triangles, its field `target_length` `left` at x0
/// and `right` at x1.
MeshData sizing_rectangle(double x0, double x1, double y0, double y1, double left, double right)
{
    MeshData sizing;
    sizing.V.resize(4, 3);
    sizing.V << x0, y0, 0, x1, y0, 0, x1, y1, 0, x0, y1, 0;
    sizing.cells[CellType::Triangle].connectivity.resize(2, 3);
    sizing.cells[CellType::Triangle].connectivity << 0, 1, 2, 0, 2, 3;
    DataMatrix<double> length(4, 1);
    length << left, right, right, left;
    sizing.vertex_attributes[SizingMesh::field_name].values = length;
    return sizing;
}

/// The box [lo, hi] as tets, its field `target_length` `left` at lo.x() and `right` at hi.x().
MeshData sizing_box(const Vector3d& lo, const Vector3d& hi, double left, double right)
{
    std::vector<Vector3d> V;
    std::vector<std::array<size_t, 4>> T;
    tet_grid(lo, hi, {{1, 1, 1}}, V, T);
    MeshData sizing;
    sizing.V.resize(Eigen::Index(V.size()), 3);
    DataMatrix<double> length(Eigen::Index(V.size()), 1);
    for (size_t v = 0; v < V.size(); ++v) {
        sizing.V.row(Eigen::Index(v)) = V[v].transpose();
        length(Eigen::Index(v), 0) = V[v].x() == lo.x() ? left : right;
    }
    auto& C = sizing.cells[CellType::Tetrahedron].connectivity;
    C.resize(Eigen::Index(T.size()), 4);
    for (size_t t = 0; t < T.size(); ++t) {
        for (int c = 0; c < 4; ++c) C(Eigen::Index(t), c) = int64_t(T[t][size_t(c)]);
    }
    sizing.vertex_attributes[SizingMesh::field_name].values = length;
    return sizing;
}

/// The column of tag `label` in a cell block's `tags`.
DataMatrix<int64_t> tag_column(const CellData& block, const std::string& label)
{
    const auto& tags = block.attributes.at("tags");
    const auto it = std::find(tags.labels.begin(), tags.labels.end(), label);
    REQUIRE(it != tags.labels.end());
    return tags.integers().col(it - tags.labels.begin());
}

/// The lengths of the square curve's edges in a 2D output, the edges between its triangles in
/// region tag_0 and those out of it: the mean on its left and on its right side, the longest.
struct CurveLengths
{
    double left = 0, right = 0, longest = 0;
};
CurveLengths curve_lengths(const MeshData& mesh)
{
    const auto& F = mesh.cell(CellType::Triangle).connectivity;
    const DataMatrix<int64_t> in = tag_column(mesh.cell(CellType::Triangle), "tag_0");
    // Per edge, whether a triangle in the region (1) and one out of it (2) have it.
    std::map<std::array<int64_t, 2>, int> sides;
    for (Eigen::Index f = 0; f < F.rows(); ++f) {
        for (int k = 0; k < 3; ++k) {
            std::array<int64_t, 2> e = {{F(f, k), F(f, (k + 1) % 3)}};
            std::sort(e.begin(), e.end());
            sides[e] |= in(f, 0) ? 1 : 2;
        }
    }
    CurveLengths out;
    int n_left = 0, n_right = 0;
    for (const auto& [e, side] : sides) {
        if (side != 3) continue;
        const Eigen::RowVector3d a = mesh.V.row(e[0]), b = mesh.V.row(e[1]);
        const double len = (a - b).norm();
        out.longest = std::max(out.longest, len);
        if (std::abs(a.x() - b.x()) > std::abs(a.y() - b.y())) continue; // the vertical sides
        const bool left = a.x() + b.x() < 1;
        (left ? out.left : out.right) += len;
        ++(left ? n_left : n_right);
    }
    REQUIRE(n_left > 0);
    REQUIRE(n_right > 0);
    out.left /= n_left;
    out.right /= n_right;
    return out;
}

/// The mean length of the edges of a 3D output's tets that lie in the slab lo <= x <= hi.
double mean_tet_edge(const MeshData& mesh, double lo, double hi)
{
    const auto& T = mesh.cell(CellType::Tetrahedron).connectivity;
    double sum = 0;
    int n = 0;
    for (Eigen::Index t = 0; t < T.rows(); ++t) {
        bool in = true;
        for (int c = 0; c < 4; ++c) in = in && mesh.V(T(t, c), 0) >= lo && mesh.V(T(t, c), 0) <= hi;
        if (!in) continue;
        for (int a = 0; a < 4; ++a) {
            for (int b = a + 1; b < 4; ++b) {
                sum += (mesh.V.row(T(t, a)) - mesh.V.row(T(t, b))).norm();
                ++n;
            }
        }
    }
    REQUIRE(n > 0);
    return sum / n;
}

/// The longest edge of a 3D output's tets.
double longest_tet_edge(const MeshData& mesh)
{
    const auto& T = mesh.cell(CellType::Tetrahedron).connectivity;
    double longest = 0;
    for (Eigen::Index t = 0; t < T.rows(); ++t) {
        for (int a = 0; a < 4; ++a) {
            for (int b = a + 1; b < 4; ++b) {
                longest = std::max(longest, (mesh.V.row(T(t, a)) - mesh.V.row(T(t, b))).norm());
            }
        }
    }
    return longest;
}

/// set_sizing_field()'s regions: `length` for the cells tagged `a`.
nlohmann::json region_a(double length)
{
    return nlohmann::json::array({{{"tags", "a"}, {"length", length}, {"length_rel", -1}}});
}

} // namespace

TEST_CASE("simwild-absolute-and-relative-lengths", "[simwild][sizing]")
{
    // The same run, by relative or by absolute values: the inputs' bounding box diagonal is
    // sqrt(2).
    const MeshMap meshes{{"curve", square_curve(20)}};
    const double diag = std::sqrt(2.0);
    const auto relative = simwild(
        {{"input", {"curve"}}, {"eps_rel", 2e-3}, {"length_rel", 0.08}, {"max_iterations", 5}},
        meshes);
    const auto absolute = simwild(
        {{"input", {"curve"}},
         {"eps", 2e-3 * diag},
         {"length", 0.08 * diag},
         {"max_iterations", 5}},
        meshes);
    CHECK(
        std::abs(relative.second["eps"].get<double>() - absolute.second["eps"].get<double>()) <
        1e-15);
    CHECK_FALSE(relative.second["sizing_mesh"].get<bool>());
    CHECK(relative.first.at("mesh").V.rows() == absolute.first.at("mesh").V.rows());
}

TEST_CASE("simwild-limit-collapse-length", "[simwild][sizing]")
{
    // A densely sampled square: its sides are collapsed down to the target length, and without
    // the limit a short edge merges into a split one, longer than the split length, which the
    // run ends on (2.02 l when measured). With it the curve's edges stay near 4/3 of the target
    // (1.21 l).
    const MeshMap meshes{{"curve", square_curve(200)}};
    const double l = 0.06;
    double longest[2] = {0, 0};
    for (const bool limit : {false, true}) {
        const auto r = simwild(
            {{"input", {"curve"}},
             {"length", l},
             {"stop_energy", 3.6},
             {"limit_collapse_length", limit}},
            meshes);
        longest[limit] = curve_lengths(r.first.at("mesh")).longest;
    }
    INFO(
        "longest curve edge without the limit " << longest[0] / l << " l, with it "
                                                << longest[1] / l << " l");
    CHECK(longest[0] > 4. / 3. * l * 1.2);
    CHECK(longest[1] <= 4. / 3. * l * 1.05);
}

TEST_CASE("simwild-limit-collapse-length-3d", "[simwild][sizing]" + release_only)
{
    // The cube's embedding, collapsed from the arrangement's fine tets: without the limit the
    // longest tet edge is 1.74 l when measured, with it 1.30 l, under 4/3. On the regular
    // background lattice: with it jittered (the default) the longest is 1.94 l and 1.42 l, the
    // limit leaving up to 1% of the edges past 4/3.
    const MeshMap meshes{{"cube", unit_cube()}};
    const double l = 0.2;
    double longest[2] = {0, 0};
    for (const bool limit : {false, true}) {
        const auto r = simwild(
            {{"input", {"cube"}},
             {"length", l},
             {"limit_collapse_length", limit},
             {"lattice_jitter", false}},
            meshes);
        longest[limit] = longest_tet_edge(r.first.at("mesh"));
    }
    INFO(
        "longest tet edge without the limit " << longest[0] / l << " l, with it " << longest[1] / l
                                              << " l");
    CHECK(longest[0] > 4. / 3. * l * 1.2);
    CHECK(longest[1] <= 4. / 3. * l * 1.05);
}

TEST_CASE("simwild-sizing-mesh-2d", "[simwild][sizing]")
{
    // A background mesh over the square whose length grows from 0.02 on the left to 0.12 on the
    // right: the curve's edges on the left side are much shorter than those on the right (0.025
    // and 0.111 when measured).
    const MeshMap meshes{
        {"curve", square_curve(10)},
        {"sizing", sizing_rectangle(-0.1, 1.1, -0.1, 1.1, 0.02, 0.12)}};
    const auto r = simwild(
        {{"input", {"curve"}}, {"length", 0.12}, {"sizing_mesh", "sizing"}, {"stop_energy", 3.6}},
        meshes);
    CHECK(r.second["sizing_mesh"].get<bool>());
    const CurveLengths c = curve_lengths(r.first.at("mesh"));
    INFO("mean curve edge on the left " << c.left << ", on the right " << c.right);
    CHECK(c.left < 0.4 * c.right);
    CHECK(c.left < 0.04);
}

TEST_CASE("simwild-sizing-mesh-3d", "[simwild][sizing]")
{
    // A tagged tet mesh of the unit cube, in a box whose length grows from 0.05 at x = -0.1 to
    // 0.25 at x = 1.1, against a global 0.25: the tets of the cube's left quarter are much smaller
    // than those of its right quarter (mean edges 0.070 and 0.204 when measured), where without
    // the field both are alike (0.205).
    const auto run = [](bool field) {
        nlohmann::json options = {{"input", {"mesh"}}, {"length", 0.25}};
        if (field) options["sizing_mesh"] = "sizing";
        const auto r = simwild(
            options,
            {{"mesh", tagged_cube(4)},
             {"sizing",
              sizing_box(Vector3d(-0.1, -0.1, -0.1), Vector3d(1.1, 1.1, 1.1), 0.05, 0.25)}});
        CHECK(r.second["sizing_mesh"].get<bool>() == field);
        const MeshData& mesh = r.first.at("mesh");
        return std::array<double, 2>{{mean_tet_edge(mesh, 0, 0.25), mean_tet_edge(mesh, 0.75, 1)}};
    };
    const auto without = run(false), with = run(true);
    INFO(
        "mean tet edge in [0, 0.25] and [0.75, 1]: without the field "
        << without[0] << ", " << without[1] << "; with it " << with[0] << ", " << with[1]);
    CHECK(with[0] < 0.5 * with[1]);
    CHECK(with[0] < 0.5 * without[0]);
}

TEST_CASE("simwild-sizing-mesh-with-regions-2d", "[simwild][sizing]")
{
    // A strip of four unit squares, the left two in region `a`; the sizing mesh covers the
    // bottom row of vertices alone. l = 1, so a sizing is a length.
    //
    //   top:    uncovered: set_sizing_field()'s, the cap at the global length included
    //   bottom: covered: the smaller of the region's and the field's, uncapped
    using Mesh = components::simwild::tri::SimWildMeshTri;
    const auto run = [](double region, double field, bool with_field) {
        components::simwild::Parameters params;
        params.init(Vector2d(-1, -1), Vector2d(5, 5));
        params.l = 1;
        Mesh mesh(params, params.eps, 0);
        std::vector<std::array<size_t, 3>> faces;
        for (size_t i = 0; i < 4; ++i) {
            faces.push_back({{i, i + 1, i + 6}});
            faces.push_back({{i, i + 6, i + 5}});
        }
        mesh.init(10, faces);
        for (size_t v = 0; v < 10; ++v) {
            auto& va = mesh.m_vertex_attribute[v];
            va.m_posf = Vector2d(double(v % 5), double(v / 5));
            va.set_pos_to_posf();
            va.m_sizing_scalar = 1;
        }
        mesh.m_tag_name_to_id["a"] = 0;
        for (const auto& f : mesh.get_faces()) {
            const auto vids = mesh.oriented_tri_vids(f);
            if (std::min({vids[0] % 5, vids[1] % 5, vids[2] % 5}) < 2) {
                mesh.m_face_attribute[f.fid(mesh)].tags = {0};
            }
        }
        mesh.set_sizing_field(region_a(region));
        if (with_field) {
            mesh.set_sizing_mesh(
                std::make_shared<SizingMesh>(
                    sizing_rectangle(-1, 5, -1, 0.5, field, field),
                    SizingMesh::Kind::Planar));
        }
        std::vector<double> s;
        for (size_t v = 0; v < 10; ++v) s.push_back(mesh.m_vertex_attribute[v].m_sizing_scalar);
        // After a pass the field only caps: a refined vertex keeps its sizing.
        if (with_field) {
            mesh.m_vertex_attribute[0].m_sizing_scalar = 0.01;
            mesh.m_vertex_attribute[4].m_sizing_scalar = 100;
            mesh.apply_sizing_mesh(false);
            CHECK(mesh.m_vertex_attribute[0].m_sizing_scalar == 0.01);
            CHECK(mesh.m_vertex_attribute[4].m_sizing_scalar == field);
        }
        return s;
    };
    for (const auto& [region, field] :
         std::vector<std::array<double, 2>>{{{0.5, 3}}, {{2, 0.25}}, {{2, 3}}}) {
        CAPTURE(region, field);
        const std::vector<double> without = run(region, field, false);
        const std::vector<double> with = run(region, field, true);
        // Top row: as without the sizing mesh, the regions' alone.
        for (size_t v = 5; v < 10; ++v) CHECK(with[v] == without[v]);
        CHECK(without[5] == region); // in the region alone
        CHECK(without[7] == std::min(region, 1.)); // on its border: capped at the global length
        CHECK(without[9] == 1); // in no region
        // Bottom row: the field where no region is, the smaller of the two where one is.
        CHECK(with[0] == std::min(region, field));
        CHECK(with[2] == std::min(region, field));
        CHECK(with[4] == field);
    }
}

TEST_CASE("simwild-sizing-mesh-with-regions-3d", "[simwild][sizing]")
{
    // The 3D twin: a row of four unit cubes, the left two in region `a`; the sizing mesh covers
    // the vertices at z = 0 alone.
    using Mesh = components::simwild::SimWildMesh;
    std::vector<Vector3d> V;
    std::vector<std::array<size_t, 4>> T;
    tet_grid(Vector3d(0, 0, 0), Vector3d(4, 1, 1), {{4, 1, 1}}, V, T);
    const auto vid = [](size_t i, size_t j, size_t k) { return (k * 2 + j) * 5 + i; };
    const auto run = [&](double region, double field, bool with_field) {
        components::simwild::Parameters params;
        params.init(Vector3d(-1, -1, -1), Vector3d(5, 5, 5));
        params.l = 1;
        Mesh mesh(params, params.eps, 0);
        mesh.init(V.size(), T);
        for (size_t v = 0; v < V.size(); ++v) {
            auto& va = mesh.m_vertex_attribute[v];
            va.m_posf = V[v];
            va.set_pos_to_posf();
            va.m_sizing_scalar = 1;
        }
        mesh.m_tag_name_to_id["a"] = 0;
        for (const auto& t : mesh.get_tets()) {
            double x = 0;
            for (const size_t v : mesh.oriented_tet_vids(t)) x += V[v].x() / 4;
            if (x < 2) mesh.m_tet_attribute[t.tid(mesh)].tags = {0};
        }
        mesh.set_sizing_field(region_a(region));
        if (with_field) {
            mesh.set_sizing_mesh(
                std::make_shared<SizingMesh>(
                    sizing_box(Vector3d(-1, -1, -1), Vector3d(5, 2, 0.5), field, field),
                    SizingMesh::Kind::Volume));
        }
        std::vector<double> s;
        for (size_t v = 0; v < V.size(); ++v) {
            s.push_back(mesh.m_vertex_attribute[v].m_sizing_scalar);
        }
        return s;
    };
    for (const auto& [region, field] :
         std::vector<std::array<double, 2>>{{{0.5, 3}}, {{2, 0.25}}, {{2, 3}}}) {
        CAPTURE(region, field);
        const std::vector<double> without = run(region, field, false);
        const std::vector<double> with = run(region, field, true);
        for (size_t j = 0; j < 2; ++j) {
            CAPTURE(j);
            for (size_t i = 0; i <= 4; ++i) CHECK(with[vid(i, j, 1)] == without[vid(i, j, 1)]);
            CHECK(without[vid(0, j, 1)] == region);
            CHECK(without[vid(2, j, 1)] == std::min(region, 1.));
            CHECK(without[vid(4, j, 1)] == 1);
            CHECK(with[vid(0, j, 0)] == std::min(region, field));
            CHECK(with[vid(2, j, 0)] == std::min(region, field));
            CHECK(with[vid(4, j, 0)] == field);
        }
    }
}

TEST_CASE("simwild-sizing-mesh-with-a-region", "[simwild][sizing]")
{
    // The square is region tag_0, at 0.08 against a global 0.16; the sizing mesh covers the
    // right half alone. On the right side of the square the smaller length wins: the field's
    // 0.02, or the region's 0.08 against a field of 0.4 -- the length the left side keeps. When
    // measured the mean curve edge is 0.091 on the left, and on the right 0.023 and 0.083.
    const auto run = [](double field) {
        const auto r = simwild(
            {{"input", {"curve"}},
             {"length", 0.16},
             {"sizing_field", nlohmann::json::array({{{"tags", "tag_0"}, {"length", 0.08}}})},
             {"sizing_mesh", "sizing"},
             {"stop_energy", 3.6}},
            {{"curve", square_curve(10)},
             {"sizing", sizing_rectangle(0.5, 1.1, -0.1, 1.1, field, field)}});
        return curve_lengths(r.first.at("mesh"));
    };
    const CurveLengths fine = run(0.02), coarse = run(0.4);
    INFO(
        "mean curve edge, field 0.02: left " << fine.left << ", right " << fine.right
                                             << "; field 0.4: left " << coarse.left << ", right "
                                             << coarse.right);
    CHECK(fine.right < 0.5 * fine.left);
    CHECK(std::abs(coarse.right - coarse.left) < 0.25 * coarse.left);
}

TEST_CASE("simwild-sizing-mesh-errors", "[simwild][sizing]")
{
    MeshData no_field;
    no_field.V.resize(3, 3);
    no_field.V << 0, 0, 0, 1, 0, 0, 0, 1, 0;
    no_field.cells[CellType::Triangle].connectivity.resize(1, 3);
    no_field.cells[CellType::Triangle].connectivity << 0, 1, 2;
    MeshMap meshes{{"curve", square_curve(4)}, {"cube", unit_cube()}, {"sizing", no_field}};
    CHECK_THROWS(simwild({{"input", {"curve"}}, {"sizing_mesh", "sizing"}}, meshes));
    CHECK_THROWS(simwild({{"input", {"curve"}}, {"sizing_mesh", "missing"}}, meshes));
    DataMatrix<double> bad(3, 1);
    bad << 0.1, -1, 0.1;
    meshes["sizing"].vertex_attributes[SizingMesh::field_name].values = bad;
    CHECK_THROWS(simwild({{"input", {"curve"}}, {"sizing_mesh", "sizing"}}, meshes));
    // A triangle mesh for a 3D run, which takes tets: thrown before the insertion.
    meshes["sizing"] = sizing_rectangle(-1, 2, -1, 2, 0.1, 0.1);
    CHECK_THROWS(simwild({{"input", {"cube"}}, {"sizing_mesh", "sizing"}}, meshes));
    // And one for a tagged tet mesh, which gives the dimension once read.
    meshes["tets"] = tagged_cube(1);
    CHECK_THROWS(simwild({{"input", {"tets"}}, {"sizing_mesh", "sizing"}}, meshes));
}

TEST_CASE("simwild-sizing-mesh-tag-operation", "[simwild][sizing]")
{
    // A tag operation does not optimize, and does not read the sizing mesh: a missing one is no
    // error.
    const auto r = simwild(
        {{"input", {"mesh"}},
         {"operation", "replace_tags"},
         {"replace_tags_in", {"a"}},
         {"replace_tags_out", {"c"}},
         {"sizing_mesh", "missing"},
         {"limit_collapse_length", true}},
        {{"mesh", tagged_cube(1)}});
    CHECK_FALSE(r.second["sizing_mesh"].get<bool>());
}

TEST_CASE("simwild-sizing-mesh-file-route", "[simwild][sizing]")
{
    // The sizing mesh as a file, relative to input_dir: the same mesh as on the memory route.
    namespace fs = std::filesystem;
    const fs::path dir = fs::current_path() / "simwild_sizing_mesh_file_route";
    fs::remove_all(dir);
    fs::create_directories(dir);
    const MeshData curve = square_curve(10);
    {
        std::ofstream o(dir / "curve.obj");
        o.precision(17);
        for (Eigen::Index v = 0; v < curve.V.rows(); ++v)
            o << "v " << curve.V(v, 0) << " " << curve.V(v, 1) << " 0\n";
        const auto& E = curve.cell(CellType::Edge).connectivity;
        for (Eigen::Index e = 0; e < E.rows(); ++e)
            o << "l " << E(e, 0) + 1 << " " << E(e, 1) + 1 << "\n";
    }
    const MeshData sizing = sizing_rectangle(-0.1, 1.1, -0.1, 1.1, 0.02, 0.12);
    io::write_mesh(dir / "sizing.msh", sizing);

    const nlohmann::json options = {{"length", 0.12}, {"stop_energy", 3.6}};
    nlohmann::json file = options;
    file.update({
        {"application", "simwild"},
        {"input", {"curve.obj"}},
        {"input_dir", dir.string()},
        {"sizing_mesh", "sizing.msh"},
        {"output", (dir / "out").string()},
    });
    simwild(file);
    nlohmann::json memory = options;
    memory.update({{"input", {"curve"}}, {"sizing_mesh", "sizing"}});
    const auto r = simwild(memory, MeshMap{{"curve", curve}, {"sizing", sizing}});
    CHECK(io::read_mesh(dir / "out.msh").V == r.first.at("mesh").V);
    fs::remove_all(dir);
}
