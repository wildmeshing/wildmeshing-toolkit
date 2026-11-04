#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/simwild/simwild.hpp>
#include <wmtk/io/tagged_mesh_data.hpp>

#include <algorithm>
#include <cmath>
#include <map>
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

/// The open square [4, 5] x [0, 1] at z = 0.5, n x n cells of two triangles.
MeshData open_square(const int n)
{
    MatrixXd V((n + 1) * (n + 1), 3);
    for (int j = 0; j <= n; ++j) {
        for (int i = 0; i <= n; ++i)
            V.row(j * (n + 1) + i) << 4 + double(i) / n, double(j) / n, 0.5;
    }
    MatrixXi F(2 * n * n, 3);
    const auto k = [n](int i, int j) { return j * (n + 1) + i; };
    for (int j = 0, r = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            F.row(r++) << k(i, j), k(i + 1, j), k(i + 1, j + 1);
            F.row(r++) << k(i, j), k(i + 1, j + 1), k(i, j + 1);
        }
    }
    return io::geometry_data(V, F, CellType::Triangle);
}

/// Distance from p to the square's rim, its four sides.
double rim_distance(const Eigen::Vector3d& p)
{
    const std::array<Eigen::Vector3d, 4> c = {{{4, 0, 0.5}, {5, 0, 0.5}, {5, 1, 0.5}, {4, 1, 0.5}}};
    double d = std::numeric_limits<double>::infinity();
    for (int s = 0; s < 4; ++s) {
        const Eigen::Vector3d a = c[s], b = c[(s + 1) % 4];
        const double t = std::clamp((p - a).dot(b - a) / (b - a).squaredNorm(), 0.0, 1.0);
        d = std::min(d, (p - (a + t * (b - a))).norm());
    }
    return d;
}

} // namespace

TEST_CASE("simwild-holds-an-open-boundary-in-its-tube", "[simwild][order2]" + release_only)
{
    // An open square next to a cube, the square kept as faces. Its rim is an open boundary, so
    // its edges are the order-2 edges tag_order2_edges() tags and holds in a tube of half-width
    // order2_envelope_ratio x eps (eps at the default). Before, only collapses checked that tube:
    // smoothing walked 3 of the 4 corners along the rim, cutting the same 0.038 at every eps_rel
    // -- 14.8 tube radii at 1e-3, 7.4 at 2e-3, 1.5 at 1e-2. The whole range, not the default alone.
    for (const double eps_rel : {1e-3, 2e-3, 1e-2}) {
        CAPTURE(eps_rel);
        const auto result = components::simwild::simwild(
            {{"input", {"cube", "square"}},
             {"input_names", {"cube", "square"}},
             {"input_type", {"volume", "surface"}},
             {"skip_simplify", true},
             {"eps_rel", eps_rel},
             {"num_threads", 1}},
            {{"cube", unit_cube()}, {"square", open_square(8)}});
        const MeshData& m = result.first.at("mesh");

        // The square's faces, and their edges with one face: the output rim.
        const CellData& faces = m.cell(CellType::Triangle);
        const MeshAttribute& tags = faces.attributes.at("tags");
        const auto column = std::find(tags.labels.begin(), tags.labels.end(), "square");
        REQUIRE(column != tags.labels.end());
        const auto member = tags.integers();
        const Eigen::Index j = column - tags.labels.begin();
        std::map<std::array<int64_t, 2>, int> n_faces;
        for (Eigen::Index r = 0; r < faces.connectivity.rows(); ++r) {
            if (member(r, j) == 0) continue;
            for (int s = 0; s < 3; ++s) {
                std::array<int64_t, 2> e = {
                    {faces.connectivity(r, s), faces.connectivity(r, (s + 1) % 3)}};
                std::sort(e.begin(), e.end());
                ++n_faces[e];
            }
        }

        const double r = eps_rel * std::sqrt(27.0); // the inputs' bbox is [0,5]x[0,1]^2
        double worst = 0;
        size_t n_rim = 0;
        for (const auto& [e, n] : n_faces) {
            if (n != 1) continue;
            ++n_rim;
            const Eigen::Vector3d a = m.V.row(e[0]).transpose(), b = m.V.row(e[1]).transpose();
            for (int s = 0; s <= 20; ++s)
                worst = std::max(worst, rim_distance(a + (b - a) * (s / 20.)));
        }
        REQUIRE(n_rim > 0);
        CAPTURE(worst / r);
        CHECK(worst <= r * (1 + 1e-9));
    }
}
