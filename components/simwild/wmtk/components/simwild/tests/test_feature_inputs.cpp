#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/simwild/simwild.hpp>
#include <wmtk/io/tagged_mesh_data.hpp>

#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <vector>

// Cases too slow for the Debug CI jobs, Windows Debug above all: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif

using namespace wmtk;

namespace {

MeshData unit_cube()
{
    MatrixXd V(8, 3);
    V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1;
    MatrixXi F(12, 3);
    F << 0, 3, 2, 0, 2, 1, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2, 0, 4, 7, 0, 7, 3,
        1, 2, 6, 1, 6, 5;
    return io::geometry_data(V, F, CellType::Triangle);
}

/// A two-segment curve inside the cube, and two vertices with no edge -- feature points by the
/// edge-file rule -- one inside the cube, one outside its bounding box.
MeshData curve_and_points()
{
    MatrixXd V(5, 3);
    V << 0.2, 0.5, 0.5, 0.5, 0.5, 0.5, 0.8, 0.8, 0.5, 0.3, 0.3, 0.7, 1.5, 0.5, 0.5;
    MatrixXi E(2, 2);
    E << 0, 1, 1, 2;
    return io::geometry_data(V, E, CellType::Edge);
}

double nearest_vertex(const MeshData& m, const Eigen::Vector3d& p)
{
    double d = std::numeric_limits<double>::infinity();
    for (Eigen::Index v = 0; v < m.V.rows(); ++v)
        d = std::min(d, (m.V.row(v).head<3>().transpose() - p).norm());
    return d;
}

/// The 2D inputs of triwild's tracking example: a circle of n segments (a region), an open wave
/// of m segments crossing it (a curve), and two points, one in the circle, one outside it.
MeshData circle_2d(const int n)
{
    MeshData c;
    c.V.resize(n, 3);
    auto& E = c.cells[CellType::Edge].connectivity;
    E.resize(n, 2);
    for (int i = 0; i < n; ++i) {
        const double a = 2 * std::acos(-1.0) * i / n;
        c.V.row(i) << std::cos(a), std::sin(a), 0;
        E.row(i) << i, (i + 1) % n;
    }
    return c;
}

MeshData wave_2d(const int m)
{
    MeshData w;
    w.V.resize(m + 1, 3);
    auto& E = w.cells[CellType::Edge].connectivity;
    E.resize(m, 2);
    for (int i = 0; i <= m; ++i) {
        const double x = -1.6 + 3.2 * i / m;
        w.V.row(i) << x, 0.3 * std::sin(6 * x) + 0.1, 0;
        if (i > 0) E.row(i - 1) << i - 1, i;
    }
    return w;
}

MeshData points_2d()
{
    MeshData p;
    p.V.resize(2, 3);
    p.V << 0.25, -0.5, 0, -0.7, 0.9, 0;
    p.cells[CellType::Point].connectivity.resize(2, 1);
    p.cells[CellType::Point].connectivity << 0, 1;
    return p;
}

double nearest_vertex_2d(const MeshData& m, const Eigen::Vector2d& p)
{
    double d = std::numeric_limits<double>::infinity();
    for (Eigen::Index v = 0; v < m.V.rows(); ++v)
        d = std::min(d, (m.V.row(v).head<2>().transpose() - p).norm());
    return d;
}

} // namespace

TEST_CASE("simwild-holds-input-feature-points", "[simwild][features]" + release_only)
{
    // The curve's ends and the two points are anchored, in a ball of feature_envelope_ratio x
    // eps (1 x eps at the default), eps as the run reports it. simwild takes eps relative to the
    // embedding's box, which is padded past the inputs and which the features grow: the point at
    // x = 1.5 lies outside the cube's padded box, and without the growth the insertion refuses
    // it. That eps is about 1.2 times eps_rel x the inputs' bbox diagonal here, where tetwild
    // takes the inputs' diagonal; the test once checked that smaller radius, which the anchors
    // do not promise, and failed on Linux at eps_rel 1e-2 (0.0245 > 0.0206, inside the run's
    // 0.0253). The whole eps range, not the default alone.
    for (const double eps_rel : {1e-3, 2e-3, 1e-2}) {
        for (const bool preserve_topology : {true, false}) {
            CAPTURE(eps_rel, preserve_topology);
            const auto result = components::simwild::simwild(
                {{"input", {"cube", "features"}},
                 {"eps_rel", eps_rel},
                 {"preserve_topology", preserve_topology},
                 {"num_threads", 1}},
                {{"cube", unit_cube()}, {"features", curve_and_points()}});
            const MeshData& m = result.first.at("mesh");
            const double eps = result.second.at("eps").get<double>();
            const MeshData f = curve_and_points();
            for (const int v : {0, 2, 3, 4}) {
                CAPTURE(v);
                CHECK(nearest_vertex(m, f.V.row(v).transpose()) <= eps * (1 + 1e-9));
            }
        }
    }
}

TEST_CASE("simwild-2d-holds-input-feature-points", "[simwild][features]" + release_only)
{
    // The 2D twin of the case above. The open curve's two ends and the two input points are
    // anchored in a ball of eps (order2_envelope_ratio and feature_envelope_ratio, 1 at the
    // default), and stay in it through the simplification after the insertion that
    // preserve_topology runs (SimWildMeshTri::simplify()), under either envelope_reference: in
    // 2D the anchors hold these points alone, and are not centred again around the simplified
    // curves. Until 2026-11 nothing simplified in 2D under preserve_topology, and "simplified"
    // was refused there. The whole eps range, not the default alone.
    const MeshData wave = wave_2d(100), points = points_2d();
    const std::vector<Eigen::Vector2d> held = {
        wave.V.row(0).head<2>().transpose(),
        wave.V.row(wave.V.rows() - 1).head<2>().transpose(),
        points.V.row(0).head<2>().transpose(),
        points.V.row(1).head<2>().transpose()};
    for (const double eps_rel : {1e-3, 2e-3, 1e-2}) {
        for (const bool preserve_topology : {true, false}) {
            for (const std::string reference : {"input", "simplified"}) {
                CAPTURE(eps_rel, preserve_topology, reference);
                const auto result = components::simwild::simwild(
                    {{"input", {"circle", "wave", "points"}},
                     {"input_type", {"volume", "curve", "point"}},
                     {"eps_rel", eps_rel},
                     {"preserve_topology", preserve_topology},
                     {"envelope_reference", reference},
                     {"length_rel", 0.05},
                     {"max_iterations", 10},
                     {"num_threads", 1}},
                    {{"circle", circle_2d(300)}, {"wave", wave}, {"points", points}});
                const MeshData& m = result.first.at("mesh");
                const double eps = result.second.at("eps").get<double>();
                for (size_t i = 0; i < held.size(); ++i) {
                    CAPTURE(i);
                    CHECK(nearest_vertex_2d(m, held[i]) <= eps * (1 + 1e-9));
                }
            }
        }
    }
}

TEST_CASE("simwild-refuses-features-without-insertion", "[simwild][features]")
{
    // A tagged tet mesh is used as it is: nothing would force the features in, so they are
    // refused rather than dropped, as every other input with a tagged mesh.
    MatrixXd V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    const MeshData tets = io::geometry_data(V, T, CellType::Tetrahedron);
    std::string what;
    try {
        components::simwild::simwild(
            {{"input", {"tets", "features"}}, {"num_threads", 1}},
            {{"tets", tets}, {"features", curve_and_points()}});
    } catch (const std::exception& e) {
        what = e.what();
    }
    CHECK(what.find("simwild's only input") != std::string::npos);
}
