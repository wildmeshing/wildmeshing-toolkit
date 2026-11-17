// The absolute eps and length, the collapse length limit and the sizing mesh.
#include <wmtk/components/triwild/triwild.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <string>
#include <vector>

using namespace wmtk;

namespace {

/// The boundary of [0, 1]^2 as one closed polyline, `n` segments per side.
MeshData square_curve(int n)
{
    MeshData m;
    m.V.resize(4 * n, 3);
    const double corners[5][2] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}};
    for (int side = 0; side < 4; ++side) {
        for (int i = 0; i < n; ++i) {
            const double t = double(i) / n;
            const auto r = side * n + i;
            m.V.row(r) << (1 - t) * corners[side][0] + t * corners[side + 1][0],
                (1 - t) * corners[side][1] + t * corners[side + 1][1], 0;
        }
    }
    auto& E = m.cells[CellType::Edge].connectivity;
    E.resize(4 * n, 2);
    for (int i = 0; i < 4 * n; ++i) E.row(i) << i, (i + 1) % (4 * n);
    return m;
}

/// Lengths of the output's tracked edges (its edge block), and of all its edges.
std::vector<double> tracked_lengths(const MeshData& mesh)
{
    std::vector<double> out;
    const auto& E = mesh.cell(CellType::Edge).connectivity;
    for (Eigen::Index i = 0; i < E.rows(); ++i) {
        out.push_back((mesh.V.row(E(i, 0)) - mesh.V.row(E(i, 1))).norm());
    }
    return out;
}

double max_of(const std::vector<double>& v)
{
    double m = 0;
    for (double x : v) m = std::max(m, x);
    return m;
}

} // namespace

TEST_CASE("triwild-absolute-eps-and-length", "[triwild][sizing]")
{
    // The same run, by relative or by absolute values: the bounding box diagonal is sqrt(2).
    MeshMap meshes{{"curve", square_curve(20)}};
    const double diag = std::sqrt(2.0);
    const auto relative = components::triwild::triwild(
        {{"input", {"curve"}}, {"eps_rel", 2e-3}, {"length_rel", 0.08}, {"max_iterations", 5}},
        meshes);
    const auto absolute = components::triwild::triwild(
        {{"input", {"curve"}},
         {"eps", 2e-3 * diag},
         {"length", 0.08 * diag},
         {"max_iterations", 5}},
        meshes);
    CHECK(
        std::abs(relative.second["eps"].get<double>() - absolute.second["eps"].get<double>()) <
        1e-15);
    CHECK(std::abs(relative.second["length"].get<double>() - 0.08 * diag) < 1e-15);
    CHECK(std::abs(absolute.second["length"].get<double>() - 0.08 * diag) < 1e-15);
    CHECK(relative.first.at("mesh").V.rows() == absolute.first.at("mesh").V.rows());
}

TEST_CASE("triwild-limit-collapse-length", "[triwild][sizing]")
{
    // A densely sampled square: its straight sides are collapsed down to the target length, and
    // without the limit a short edge merges into a split one, longer than the split length, which
    // the run ends on (2.16 l when measured). With it the tracked edges stay near 4/3 of the
    // target (smoothing may still stretch them a little: 1.35 l).
    MeshMap meshes{{"curve", square_curve(200)}};
    const double l = 0.06;
    double longest[2] = {0, 0};
    for (const bool limit : {false, true}) {
        const auto r = components::triwild::triwild(
            {{"input", {"curve"}},
             {"length", l},
             {"stop_energy", 3.6},
             {"limit_collapse_length", limit},
             {"filter", "input"},
             {"preserve_topology", true}},
            meshes);
        longest[limit] = max_of(tracked_lengths(r.first.at("mesh")));
    }
    INFO(
        "longest tracked edge without the limit " << longest[0] / l << " l, with it "
                                                  << longest[1] / l << " l");
    CHECK(longest[0] > 4. / 3. * l * 1.2);
    CHECK(longest[1] <= 4. / 3. * l * 1.05);
}

TEST_CASE("triwild-sizing-mesh", "[triwild][sizing]")
{
    // A background mesh over the square whose length grows from 0.02 on the left to 0.12 on the
    // right: the tracked edges on the left side are much shorter than those on the right.
    MeshData sizing;
    sizing.V.resize(4, 3);
    sizing.V << -0.1, -0.1, 0, 1.1, -0.1, 0, 1.1, 1.1, 0, -0.1, 1.1, 0;
    sizing.cells[CellType::Triangle].connectivity.resize(2, 3);
    sizing.cells[CellType::Triangle].connectivity << 0, 1, 2, 0, 2, 3;
    DataMatrix<double> length(4, 1);
    length << 0.02, 0.12, 0.12, 0.02;
    sizing.vertex_attributes["length"].values = length;
    MeshMap meshes{{"curve", square_curve(10)}, {"sizing", sizing}};

    const auto r = components::triwild::triwild(
        {{"input", {"curve"}},
         {"length", 0.12},
         {"sizing_mesh", "sizing"},
         {"stop_energy", 3.6},
         {"filter", "input"},
         {"preserve_topology", true}},
        meshes);
    CHECK(r.second["sizing_mesh"].get<bool>());
    const MeshData& mesh = r.first.at("mesh");
    const auto& E = mesh.cell(CellType::Edge).connectivity;
    double left = 0, right = 0;
    int n_left = 0, n_right = 0;
    for (Eigen::Index i = 0; i < E.rows(); ++i) {
        const Eigen::RowVector3d a = mesh.V.row(E(i, 0)), b = mesh.V.row(E(i, 1));
        const double x = 0.5 * (a.x() + b.x()), len = (a - b).norm();
        if (std::abs(a.x() - b.x()) > 1e-9) continue; // the vertical sides only
        (x < 0.5 ? left : right) += len;
        ++(x < 0.5 ? n_left : n_right);
    }
    REQUIRE(n_left > 0);
    REQUIRE(n_right > 0);
    INFO("mean tracked edge on the left " << left / n_left << ", on the right " << right / n_right);
    CHECK(left / n_left < 0.4 * (right / n_right));
    CHECK(left / n_left < 0.04);
}

TEST_CASE("triwild-sizing-mesh-errors", "[triwild][sizing]")
{
    MeshData no_field;
    no_field.V.resize(3, 3);
    no_field.V << 0, 0, 0, 1, 0, 0, 0, 1, 0;
    no_field.cells[CellType::Triangle].connectivity.resize(1, 3);
    no_field.cells[CellType::Triangle].connectivity << 0, 1, 2;
    MeshMap meshes{{"curve", square_curve(4)}, {"sizing", no_field}};
    CHECK_THROWS(
        components::triwild::triwild({{"input", {"curve"}}, {"sizing_mesh", "sizing"}}, meshes));
    CHECK_THROWS(
        components::triwild::triwild({{"input", {"curve"}}, {"sizing_mesh", "missing"}}, meshes));
    DataMatrix<double> bad(3, 1);
    bad << 0.1, -1, 0.1;
    meshes["sizing"].vertex_attributes["length"].values = bad;
    CHECK_THROWS(
        components::triwild::triwild({{"input", {"curve"}}, {"sizing_mesh", "sizing"}}, meshes));
}
