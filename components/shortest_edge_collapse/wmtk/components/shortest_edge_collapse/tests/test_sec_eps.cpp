// The absolute envelope thickness, eps.
#include <wmtk/components/shortest_edge_collapse/shortest_edge_collapse.hpp>

#include <igl/PI.h>
#include <igl/point_mesh_squared_distance.h>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>

using namespace wmtk;
using components::shortest_edge_collapse::shortest_edge_collapse;

namespace {

/// A torus with bumps on its tube, n x m vertices: closed, so no vertex is frozen, and its bumps
/// are what an envelope keeps collapses from cutting.
MeshData bumpy_torus(int n, int m)
{
    MeshData t;
    t.V.resize(n * m, 3);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            const double u = 2 * igl::PI * i / n, v = 2 * igl::PI * j / m;
            const double r = 0.4 + 0.05 * std::sin(5 * u) * std::sin(3 * v);
            t.V.row(i * m + j) << (1 + r * std::cos(v)) * std::cos(u),
                (1 + r * std::cos(v)) * std::sin(u), r * std::sin(v);
        }
    }
    auto& F = t.cells[CellType::Triangle].connectivity;
    F.resize(2 * n * m, 3);
    const auto id = [n, m](int i, int j) { return int64_t((i % n) * m + j % m); };
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            F.row(2 * (i * m + j)) << id(i, j), id(i + 1, j), id(i + 1, j + 1);
            F.row(2 * (i * m + j) + 1) << id(i, j), id(i + 1, j + 1), id(i, j + 1);
        }
    }
    return t;
}

/// The bounding box diagonal, as shortest_edge_collapse computes it.
double diagonal(const MeshData& mesh)
{
    const Eigen::MatrixXd V = mesh.V;
    const Eigen::MatrixXd box_min = V.colwise().minCoeff();
    const Eigen::MatrixXd box_max = V.colwise().maxCoeff();
    return (box_max - box_min).norm();
}

/// The farthest the output gets from the input, sampled at the output's vertices, edge midpoints
/// and triangle centroids.
double distance_to(const MeshData& output, const MeshData& input)
{
    const Eigen::MatrixXd V = input.V;
    const Eigen::MatrixXi F = input.cell(CellType::Triangle).connectivity.cast<int>();
    const auto& T = output.cell(CellType::Triangle).connectivity;
    Eigen::MatrixXd P(7 * T.rows(), 3);
    for (Eigen::Index f = 0; f < T.rows(); ++f) {
        const Eigen::RowVector3d a = output.V.row(T(f, 0)), b = output.V.row(T(f, 1)),
                                 c = output.V.row(T(f, 2));
        P.middleRows(7 * f, 7) << a, b, c, (a + b) / 2, (b + c) / 2, (c + a) / 2, (a + b + c) / 3;
    }
    Eigen::VectorXd d2;
    Eigen::VectorXi I;
    Eigen::MatrixXd C;
    igl::point_mesh_squared_distance(P, V, F, d2, I, C);
    return std::sqrt(d2.maxCoeff());
}

} // namespace

TEST_CASE("sec-absolute-eps", "[test_sec][eps]")
{
    // eps_rel times the diagonal, given as eps, is the same run: the same output mesh, and the
    // same thickness in the report. A positive eps wins over eps_rel.
    MeshMap meshes{{"torus", bumpy_torus(48, 16)}};
    const double eps = 5e-3 * diagonal(meshes.at("torus"));
    const auto relative = shortest_edge_collapse({{"input", "torus"}, {"eps_rel", 5e-3}}, meshes);
    for (const double eps_rel : {-1.0, 5e-2}) {
        const auto absolute = shortest_edge_collapse(
            {{"input", "torus"}, {"eps", eps}, {"eps_rel", eps_rel}},
            meshes);
        CHECK(relative.second.at("eps").get<double>() == eps);
        CHECK(absolute.second.at("eps").get<double>() == eps);
        const MeshData& r = relative.first.at("mesh");
        const MeshData& a = absolute.first.at("mesh");
        CHECK(r.V == a.V);
        CHECK(r.cell(CellType::Triangle).connectivity == a.cell(CellType::Triangle).connectivity);
    }
}

TEST_CASE("sec-absolute-eps-is-honored", "[test_sec][eps]")
{
    // eps alone, eps_rel at its default (no envelope), holds the output within eps of the input
    // and stops the collapses short of the target; without it they go to the target, and cut the
    // bumps by more than eps.
    MeshMap meshes{{"torus", bumpy_torus(48, 16)}};
    const MeshData& input = meshes.at("torus");
    const double eps = 5e-3 * diagonal(input);
    const auto held = shortest_edge_collapse({{"input", "torus"}, {"eps", eps}}, meshes);
    const auto bare = shortest_edge_collapse({{"input", "torus"}}, meshes);
    const double d_held = distance_to(held.first.at("mesh"), input);
    const double d_bare = distance_to(bare.first.at("mesh"), input);
    const size_t n_held = held.second.at("vertices"), n_bare = bare.second.at("vertices");
    INFO(
        "with eps: " << n_held << " vertices, " << d_held / eps << " eps from the input; without: "
                     << n_bare << " vertices, " << d_bare / eps << " eps");
    CHECK(d_held <= eps);
    CHECK(d_bare > eps);
    CHECK(n_held > n_bare);
}
