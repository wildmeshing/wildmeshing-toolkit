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

} // namespace

TEST_CASE("simwild-holds-input-feature-points", "[simwild][features]" + release_only)
{
    // The curve's ends and the two points are anchored, in a ball of feature_envelope_ratio x
    // eps. eps is relative to the embedding's box, which the features grow: the point at x = 1.5
    // lies outside the cube's padded box, and without the growth the insertion refuses it. The
    // radius checked, eps_rel x the inputs' bbox diagonal, is below the anchors' own. The whole
    // eps range, not the default alone.
    const double diag = std::sqrt(1.5 * 1.5 + 2.0); // the inputs' bbox is [0,1.5]x[0,1]^2
    for (const double eps_rel : {1e-3, 2e-3, 1e-2}) {
        for (const bool preserve_topology : {true, false}) {
            CAPTURE(eps_rel, preserve_topology);
            const auto result = components::simwild::simwild(
                {{"input", {"cube"}},
                 {"input_edges", {"features"}},
                 {"eps_rel", eps_rel},
                 {"preserve_topology", preserve_topology},
                 {"num_threads", 1}},
                {{"cube", unit_cube()}, {"features", curve_and_points()}});
            const MeshData& m = result.first.at("mesh");
            const MeshData f = curve_and_points();
            for (const int v : {0, 2, 3, 4}) {
                CAPTURE(v);
                CHECK(nearest_vertex(m, f.V.row(v).transpose()) <= eps_rel * diag);
            }
        }
    }
}

TEST_CASE("simwild-refuses-features-without-insertion", "[simwild][features]")
{
    // A tagged tet mesh is used as it is: nothing would force the features in, so they are
    // refused rather than dropped.
    MatrixXd V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    const MeshData tets = io::geometry_data(V, T, CellType::Tetrahedron);
    std::string what;
    try {
        components::simwild::simwild(
            {{"input", {"tets"}}, {"input_edges", {"features"}}, {"num_threads", 1}},
            {{"tets", tets}, {"features", curve_and_points()}});
    } catch (const std::exception& e) {
        what = e.what();
    }
    CHECK(
        what.find("input_edges and input_points apply to inserted inputs only") !=
        std::string::npos);
}
