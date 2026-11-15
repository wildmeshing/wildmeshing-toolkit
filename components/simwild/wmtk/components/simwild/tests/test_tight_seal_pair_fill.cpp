#include <wmtk/components/simwild/TightSealPairFill.hpp>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <random>
#include <vector>

using namespace wmtk::components::simwild;

// tight_seal::PairFill, the fill of tight_seal_geom (tight seal v2 note, section 4.2), on a soup of
// cells with linear functions: then the values PairFill interpolates are the functions' own, so
// every label can be checked against them.

namespace {

// Cells of N corners over points in N - 1 dimensions, with the edge split PairFill needs: each
// cell on the edge is replaced by its two halves, the first taking the cell's id (as the mesh's
// slots are reused), the second a new one.
template <int N>
struct Soup
{
    using Point = Eigen::Matrix<double, N - 1, 1>;
    std::vector<Point> points;
    std::vector<std::array<size_t, N>> cells;
    std::vector<char> alive;

    size_t split(
        const size_t a,
        const size_t b,
        const double delta_a,
        const double delta_b,
        std::vector<std::pair<size_t, size_t>>& children)
    {
        children.clear();
        const double s = delta_a / (delta_a - delta_b);
        const size_t x = points.size();
        points.push_back(points[a] + s * (points[b] - points[a]));
        const size_t n = cells.size();
        for (size_t id = 0; id < n; ++id) {
            if (!alive[id]) {
                continue;
            }
            const std::array<size_t, N> cell = cells[id];
            const auto ia = std::find(cell.begin(), cell.end(), a);
            const auto ib = std::find(cell.begin(), cell.end(), b);
            if (ia == cell.end() || ib == cell.end()) {
                continue;
            }
            // x on the segment keeps each half's orientation
            std::array<size_t, N> keeps_a = cell, keeps_b = cell;
            keeps_a[ib - cell.begin()] = x;
            keeps_b[ia - cell.begin()] = x;
            cells[id] = keeps_a;
            children.emplace_back(id, id);
            cells.push_back(keeps_b);
            alive.push_back(1);
            children.emplace_back(id, cells.size() - 1);
        }
        return x;
    }

    /// (N - 1)! times the signed measure
    double measure(const size_t id) const
    {
        Eigen::Matrix<double, N - 1, N - 1> m;
        for (int q = 1; q < N; ++q) {
            m.col(q - 1) = points[cells[id][q]] - points[cells[id][0]];
        }
        return m.determinant();
    }

    Point centroid(const size_t id) const
    {
        Point p = Point::Zero();
        for (const size_t v : cells[id]) {
            p += points[v] / double(N);
        }
        return p;
    }
};

// [0, n]^2 in unit squares, each in two triangles, counterclockwise
Soup<3> grid_2d(const int n)
{
    Soup<3> soup;
    for (int j = 0; j <= n; ++j) {
        for (int i = 0; i <= n; ++i) {
            soup.points.emplace_back(i, j);
        }
    }
    const auto id = [&](const int i, const int j) { return size_t(j * (n + 1) + i); };
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            const size_t a = id(i, j), b = id(i + 1, j), c = id(i + 1, j + 1), d = id(i, j + 1);
            if ((i + j) % 2 == 0) {
                soup.cells.push_back({{a, b, c}});
                soup.cells.push_back({{a, c, d}});
            } else {
                soup.cells.push_back({{a, b, d}});
                soup.cells.push_back({{b, c, d}});
            }
        }
    }
    soup.alive.assign(soup.cells.size(), 1);
    return soup;
}

// [0, n]^3 in unit cubes, each in six tets around its diagonal, positively oriented
Soup<4> grid_3d(const int n)
{
    Soup<4> soup;
    for (int k = 0; k <= n; ++k) {
        for (int j = 0; j <= n; ++j) {
            for (int i = 0; i <= n; ++i) {
                soup.points.emplace_back(i, j, k);
            }
        }
    }
    const auto id = [&](const int i, const int j, const int k) {
        return size_t((k * (n + 1) + j) * (n + 1) + i);
    };
    std::array<int, 3> axes = {{0, 1, 2}};
    for (int k = 0; k < n; ++k) {
        for (int j = 0; j < n; ++j) {
            for (int i = 0; i < n; ++i) {
                std::sort(axes.begin(), axes.end());
                do {
                    std::array<int, 3> c = {{i, j, k}};
                    std::array<size_t, 4> tet;
                    tet[0] = id(c[0], c[1], c[2]);
                    for (int q = 0; q < 3; ++q) {
                        ++c[axes[q]];
                        tet[q + 1] = id(c[0], c[1], c[2]);
                    }
                    soup.cells.push_back(tet);
                    if (soup.measure(soup.cells.size() - 1) < 0) {
                        std::swap(soup.cells.back()[2], soup.cells.back()[3]);
                    }
                } while (std::next_permutation(axes.begin(), axes.end()));
            }
        }
    }
    soup.alive.assign(soup.cells.size(), 1);
    return soup;
}

// K linear functions g_k . p + c_k
template <int N>
struct Linear
{
    std::vector<Eigen::Matrix<double, N - 1, 1>> g;
    std::vector<double> c;
    double operator()(const size_t k, const Eigen::Matrix<double, N - 1, 1>& p) const
    {
        return g[k].dot(p) + c[k];
    }
};

// Random functions; with `integers`, small integer gradients and constants, so many vertices of
// the grid tie, and some functions may be equal everywhere.
template <int N>
Linear<N> random_functions(std::mt19937& rng, const size_t K, const bool integers, const int n)
{
    Linear<N> f;
    std::uniform_real_distribution<double> real(-1., 1.);
    std::uniform_int_distribution<int> small(-1, 1);
    std::uniform_int_distribution<int> offset(0, n);
    for (size_t k = 0; k < K; ++k) {
        Eigen::Matrix<double, N - 1, 1> g;
        for (int d = 0; d < N - 1; ++d) {
            g[d] = integers ? double(small(rng)) : real(rng);
        }
        f.g.push_back(g);
        f.c.push_back(integers ? double(offset(rng)) : real(rng) * n);
    }
    return f;
}

// Runs PairFill on the soup, components[c] its cells and fs[c] its functions, and checks: no split
// refused and every cell of the components finalized; each labelled with a function smallest at
// its centroid and at its corners (up to rounding); the cells outside the components unlabelled;
// the measure kept.
template <int N>
void run_and_check(
    Soup<N>& soup,
    const std::vector<std::vector<size_t>>& components,
    const std::vector<Linear<N>>& fs)
{
    double before = 0;
    for (size_t id = 0; id < soup.cells.size(); ++id) {
        before += soup.measure(id);
    }
    const size_t n_points = soup.points.size();
    typename tight_seal::PairFill<N>::Mesh mesh;
    mesh.corners = [&](const size_t id) { return soup.cells[id]; };
    mesh.value = [&](const size_t c, const size_t k, const size_t v) {
        REQUIRE(v < n_points); // only the vertices there before the fill
        return fs[c](k, soup.points[v]);
    };
    mesh.split = [&](const size_t a,
                     const size_t b,
                     const double delta_a,
                     const double delta_b,
                     std::vector<std::pair<size_t, size_t>>& children) {
        return soup.split(a, b, delta_a, delta_b, children);
    };
    std::vector<size_t> n_functions;
    for (const Linear<N>& f : fs) {
        n_functions.push_back(f.g.size());
    }
    tight_seal::PairFill<N> fill(mesh, components, n_functions);
    fill.run();

    CHECK(fill.stats().n_refused == 0);
    CHECK(fill.stats().n_fallback == 0);
    double after = 0;
    size_t n_labelled = 0;
    for (size_t id = 0; id < soup.cells.size(); ++id) {
        if (!soup.alive[id]) {
            continue;
        }
        after += soup.measure(id);
        const int c = fill.component(id);
        if (c < 0) {
            continue;
        }
        ++n_labelled;
        const int label = fill.label(id);
        REQUIRE(label >= 0);
        const Linear<N>& f = fs[size_t(c)];
        const auto smallest_at = [&](const Eigen::Matrix<double, N - 1, 1>& p) {
            for (size_t k = 0; k < f.g.size(); ++k) {
                if (f(size_t(label), p) > f(k, p) + 1e-9) {
                    return false;
                }
            }
            return true;
        };
        CHECK(smallest_at(soup.centroid(id)));
        for (const size_t v : soup.cells[id]) {
            CHECK(smallest_at(soup.points[v]));
        }
    }
    CHECK(n_labelled == fill.stats().n_cells);
    CHECK(std::abs(after - before) <= 1e-9 * std::abs(before));
}

} // namespace

TEST_CASE("tight_seal_pair_fill", "[simwild][tight_seal_geom][tight_seal_pair_fill]")
{
    std::mt19937 rng(7);

    SECTION("triangles, one component")
    {
        for (int trial = 0; trial < 60; ++trial) {
            const bool integers = trial % 2 == 1;
            const size_t K = 2 + size_t(trial % 5);
            Soup<3> soup = grid_2d(4);
            std::vector<size_t> all(soup.cells.size());
            for (size_t id = 0; id < all.size(); ++id) {
                all[id] = id;
            }
            run_and_check<3>(soup, {all}, {random_functions<3>(rng, K, integers, 4)});
        }
    }

    SECTION("tets, one component")
    {
        for (int trial = 0; trial < 40; ++trial) {
            const bool integers = trial % 2 == 1;
            const size_t K = 2 + size_t(trial % 4);
            Soup<4> soup = grid_3d(2);
            std::vector<size_t> all(soup.cells.size());
            for (size_t id = 0; id < all.size(); ++id) {
                all[id] = id;
            }
            run_and_check<4>(soup, {all}, {random_functions<4>(rng, K, integers, 2)});
        }
    }

    SECTION("two components with their own functions, and cells in neither")
    {
        // Component 0 left of x = 2, component 1 right of it, either touching the other (a split
        // on their interface makes a vertex the other must interpolate with its own functions)
        // or with a band of cells in neither between them (1 < x < 2). Each one's splits cut the
        // cells beside it.
        for (int trial = 0; trial < 24; ++trial) {
            const bool integers = trial % 2 == 1;
            const bool band = trial % 4 >= 2;
            Soup<4> soup = grid_3d(4);
            std::vector<std::vector<size_t>> components(2);
            for (size_t id = 0; id < soup.cells.size(); ++id) {
                const double x = soup.centroid(id)[0];
                if (x < (band ? 1 : 2)) {
                    components[0].push_back(id);
                } else if (x > 2) {
                    components[1].push_back(id);
                }
            }
            run_and_check<4>(
                soup,
                components,
                {random_functions<4>(rng, 3, integers, 4),
                 random_functions<4>(rng, 4, integers, 4)});
        }
    }

    SECTION("functions equal everywhere tie, to the smaller")
    {
        Soup<3> soup = grid_2d(3);
        std::vector<size_t> all(soup.cells.size());
        for (size_t id = 0; id < all.size(); ++id) {
            all[id] = id;
        }
        Linear<3> f;
        f.g = {Eigen::Vector2d(1, 0), Eigen::Vector2d(-1, 0), Eigen::Vector2d(1, 0)};
        f.c = {0., 3., 0.}; // 0 and 2 the same; 1 smallest for x > 1.5
        run_and_check<3>(soup, {all}, {f});
    }
}
