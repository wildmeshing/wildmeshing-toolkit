// VisibleField (EXPERIMENTAL_visible_distance) on synthetic tet grids: two input walls with a gap,
// band cells attached to wall A, outside cells, band cells attached to wall B. Grid boxes split
// into 6 Kuhn tets, so segments along grid lines pass exactly through mesh edges and vertices.
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/topological_offset/VisibleField.hpp>
#include <wmtk/utils/predicates.hpp>

#include <Eigen/Dense>
#include <functional>
#include <map>
#include <random>
#include <set>

using namespace wmtk::components::topological_offset;
using Kind = VisibilityCells::Kind;

namespace {

struct Grid : VisibilityCells
{
    std::vector<Eigen::Vector3d> P;
    std::vector<std::array<int64_t, 4>> T;
    std::vector<Kind> K;
    std::map<std::array<int64_t, 3>, std::vector<int64_t>> face_cells;
    std::map<std::pair<int64_t, int64_t>, std::vector<int64_t>> edge_cells;
    std::map<int64_t, std::vector<int64_t>> vert_cells;

    Grid(
        const std::vector<double>& X,
        const std::vector<double>& Y,
        const std::vector<double>& Z,
        const std::function<Kind(const Eigen::Vector3d&)>& kind_at)
    {
        const auto vid = [&](size_t i, size_t j, size_t k) {
            return int64_t((i * Y.size() + j) * Z.size() + k);
        };
        for (double x : X)
            for (double y : Y)
                for (double z : Z) P.emplace_back(x, y, z);
        const std::array<std::array<int, 3>, 6> perms = {
            {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}}};
        for (size_t i = 0; i + 1 < X.size(); ++i)
            for (size_t j = 0; j + 1 < Y.size(); ++j)
                for (size_t k = 0; k + 1 < Z.size(); ++k) {
                    const Eigen::Vector3d c(
                        (X[i] + X[i + 1]) / 2,
                        (Y[j] + Y[j + 1]) / 2,
                        (Z[k] + Z[k + 1]) / 2);
                    const Kind kd = kind_at(c);
                    for (const auto& perm : perms) {
                        std::array<size_t, 3> d = {0, 0, 0};
                        std::array<int64_t, 4> t;
                        t[0] = vid(i, j, k);
                        for (int s = 0; s < 3; ++s) {
                            d[size_t(perm[size_t(s)])] = 1;
                            t[size_t(s) + 1] = vid(i + d[0], j + d[1], k + d[2]);
                        }
                        T.push_back(t);
                        K.push_back(kd);
                    }
                }
        for (int64_t c = 0; c < int64_t(T.size()); ++c) {
            const auto& t = T[size_t(c)];
            for (int j = 0; j < 4; ++j) {
                std::array<int64_t, 3> f;
                int n = 0;
                for (int i = 0; i < 4; ++i)
                    if (i != j) f[size_t(n++)] = t[size_t(i)];
                std::sort(f.begin(), f.end());
                face_cells[f].push_back(c);
            }
            for (int a = 0; a < 4; ++a) {
                vert_cells[t[size_t(a)]].push_back(c);
                for (int b = a + 1; b < 4; ++b) {
                    edge_cells[{std::min(t[size_t(a)], t[size_t(b)]),
                                std::max(t[size_t(a)], t[size_t(b)])}]
                        .push_back(c);
                }
            }
        }
    }
    std::array<int64_t, 4> vertices(int64_t c) const override { return T[size_t(c)]; }
    Eigen::Vector3d position(int64_t v) const override { return P[size_t(v)]; }
    Kind kind(int64_t c) const override { return K[size_t(c)]; }
    int64_t neighbor(int64_t c, int j) const override
    {
        std::array<int64_t, 3> f;
        int n = 0;
        for (int i = 0; i < 4; ++i)
            if (i != j) f[size_t(n++)] = T[size_t(c)][size_t(i)];
        std::sort(f.begin(), f.end());
        for (int64_t o : face_cells.at(f))
            if (o != c) return o;
        return -1;
    }
    void cells_around_edge(int64_t a, int64_t b, std::vector<int64_t>& out) const override
    {
        out = edge_cells.at({std::min(a, b), std::max(a, b)});
    }
    void cells_around_vertex(int64_t v, std::vector<int64_t>& out) const override
    {
        out = vert_cells.at(v);
    }

    int64_t vertex_at(const Eigen::Vector3d& x) const
    {
        for (int64_t v = 0; v < int64_t(P.size()); ++v)
            if ((P[size_t(v)] - x).norm() < 1e-12) return v;
        return -1;
    }
    /// The start of a front vertex: its band cells, each with its three faces through it.
    VisibleStart vertex_start(int64_t v) const
    {
        VisibleStart s;
        for (int64_t c : vert_cells.at(v)) {
            if (K[size_t(c)] != Kind::Band) continue;
            uint8_t mask = 0;
            for (int j = 0; j < 4; ++j)
                if (T[size_t(c)][size_t(j)] != v) mask |= uint8_t(1u << j);
            s.cells.push_back({c, mask});
        }
        return s;
    }
    /// The start of a point inside front face f (sorted vertex ids): f's band cell, that face.
    VisibleStart face_start(
        std::array<int64_t, 3> f,
        const std::array<double, 3>& w = {{1., 1., 1.}}) const
    {
        VisibleStart s;
        for (int i = 0; i < 3; ++i) s.support.push_back({f[size_t(i)], w[size_t(i)]});
        std::sort(f.begin(), f.end());
        for (int64_t c : face_cells.at(f)) {
            if (K[size_t(c)] != Kind::Band) continue;
            for (int j = 0; j < 4; ++j) {
                const int64_t opp = T[size_t(c)][size_t(j)];
                if (opp != f[0] && opp != f[1] && opp != f[2])
                    s.cells.push_back({c, uint8_t(1u << j)});
            }
        }
        return s;
    }
    /// A front face with all corners on the plane x = x0 inside the box y in [y0,y1], z in [z0,z1].
    std::array<int64_t, 3> front_face(double x0, double y0, double y1, double z0, double z1) const
    {
        for (const auto& [f, cs] : face_cells) {
            if (cs.size() != 2) continue;
            const Kind a = K[size_t(cs[0])], b = K[size_t(cs[1])];
            if (!((a == Kind::Band) != (b == Kind::Band))) continue;
            bool ok = true;
            for (int64_t v : f) {
                const auto& x = P[size_t(v)];
                ok = ok && std::abs(x[0] - x0) < 1e-12 && x[1] >= y0 - 1e-12 &&
                     x[1] <= y1 + 1e-12 && x[2] >= z0 - 1e-12 && x[2] <= z1 + 1e-12;
            }
            if (ok) return f;
        }
        return {-1, -1, -1};
    }
};

/// Two input walls facing a gap: wall A's face on x = xa (solid below), wall B's on x = xb
/// (solid above), each as two triangles over y, z in [0, 2].
VisibleField walls(double xa, double xb, double delta)
{
    std::vector<Eigen::Vector3d> V = {
        {xa, 0, 0},
        {xa, 2, 0},
        {xa, 2, 2},
        {xa, 0, 2},
        {xb, 0, 0},
        {xb, 2, 0},
        {xb, 2, 2},
        {xb, 0, 2}};
    std::vector<Eigen::Vector3i> F = {{0, 1, 2}, {0, 2, 3}, {4, 5, 6}, {4, 6, 7}};
    std::vector<int8_t> inner;
    const Eigen::Vector3d ina(xa - 1, 1, 1), inb(xb + 1, 1, 1);
    for (size_t t = 0; t < F.size(); ++t) {
        const auto& f = F[t];
        const Eigen::Vector3d& in = t < 2 ? ina : inb;
        inner.push_back(int8_t(
            wmtk::utils::predicates::orient3d(
                V[size_t(f[0])],
                V[size_t(f[1])],
                V[size_t(f[2])],
                in)));
    }
    return VisibleField(V, F, inner, delta);
}

} // namespace

TEST_CASE("visible-field-past-the-midline", "[offset][3d][visible]")
{
    // Gap x in (-1, 1), midline 0. Band A on [-1, 0.25] has crossed the midline, outside on
    // (0.25, 0.5), band B on [0.5, 1]. A point of band A's front (x = 0.25) is 0.75 from wall B
    // and 1.25 from wall A: the euclidean distance picks wall B, through the outside layer; the
    // visible distance is wall A's.
    const Grid g(
        {-3, -1, -0.5, 0.25, 0.5, 1, 3},
        {0, 1, 2},
        {0, 1, 2},
        [](const Eigen::Vector3d& c) {
            if (c[0] < -1 || c[0] > 1) return Kind::Input;
            if (c[0] < 0.25) return Kind::Band;
            if (c[0] < 0.5) return Kind::Other;
            return Kind::Band;
        });
    const VisibleField vf = walls(-1, 1, 1.35);
    // A point inside a front face.
    const auto f = g.front_face(0.25, 0, 1, 0, 1);
    REQUIRE(f[0] >= 0);
    const Eigen::Vector3d p = (g.P[size_t(f[0])] + g.P[size_t(f[1])] + g.P[size_t(f[2])]) / 3.;
    const auto r = vf.nearest(p, g.face_start(f), g);
    REQUIRE(r.status == VisibleField::Feature::Status::Found);
    CHECK(r.d == Catch::Approx(1.25));
    CHECK(r.foot[0] == Catch::Approx(-1.));
    CHECK(vf.relative_residual(p, r) == Catch::Approx((1.25 - 1.35) / 1.35));
    // A front vertex: the segment to wall A runs along the grid line through the vertex
    // (-0.5, 1, 1), a degenerate exit through a vertex.
    const int64_t v = g.vertex_at({0.25, 1, 1});
    REQUIRE(v >= 0);
    const auto rv = vf.nearest(g.P[size_t(v)], g.vertex_start(v), g);
    REQUIRE(rv.status == VisibleField::Feature::Status::Found);
    CHECK(rv.d == Catch::Approx(1.25));
    CHECK(rv.foot[0] == Catch::Approx(-1.));
    // Band B's front (x = 0.5) sees wall B at 0.5.
    const int64_t w = g.vertex_at({0.5, 1, 1});
    const auto rw = vf.nearest(g.P[size_t(w)], g.vertex_start(w), g);
    REQUIRE(rw.status == VisibleField::Feature::Status::Found);
    CHECK(rw.d == Catch::Approx(0.5));
    CHECK(rw.foot[0] == Catch::Approx(1.));
}

TEST_CASE("visible-field-same-as-euclidean-when-visible", "[offset][3d][visible]")
{
    // Band A on [-1, -0.25]: its front is nearer wall A, and the segment to it stays in the band.
    const Grid g(
        {-3, -1, -0.5, -0.25, 0.25, 1, 3},
        {0, 1, 2},
        {0, 1, 2},
        [](const Eigen::Vector3d& c) {
            if (c[0] < -1 || c[0] > 1) return Kind::Input;
            if (c[0] < -0.25) return Kind::Band;
            if (c[0] < 0.25) return Kind::Other;
            return Kind::Band;
        });
    const VisibleField vf = walls(-1, 1, 1.0);
    const int64_t v = g.vertex_at({-0.25, 1, 1});
    const auto r = vf.nearest(g.P[size_t(v)], g.vertex_start(v), g);
    REQUIRE(r.status == VisibleField::Feature::Status::Found);
    CHECK(r.d == Catch::Approx(0.75));
}

TEST_CASE("visible-field-ends-in-input-cells", "[offset][3d][visible]")
{
    // The mesh's input cells stand 0.01 off wall A's triangles (x = -0.99 against -1): the
    // segment reaches q through an input cell, which ends a walk legally.
    const Grid g(
        {-3, -0.99, -0.5, 0.25, 0.5, 1, 3},
        {0, 1, 2},
        {0, 1, 2},
        [](const Eigen::Vector3d& c) {
            if (c[0] < -0.99 || c[0] > 1) return Kind::Input;
            if (c[0] < 0.25) return Kind::Band;
            if (c[0] < 0.5) return Kind::Other;
            return Kind::Band;
        });
    const VisibleField vf = walls(-1, 1, 1.35);
    const int64_t v = g.vertex_at({0.25, 1, 1});
    const auto r = vf.nearest(g.P[size_t(v)], g.vertex_start(v), g);
    REQUIRE(r.status == VisibleField::Feature::Status::Found);
    CHECK(r.d == Catch::Approx(1.25));
}

TEST_CASE("visible-field-lower-bound-on-a-partly-hidden-triangle", "[offset][3d][visible]")
{
    // A pocket of outside cells inside band A, x in [-0.5, 0], y in [0, 1], z in [0, 1], sits
    // between a front point at y, z = 0.5 and its foot on wall A. Wall B is hidden by the outside
    // layer. Wall A's triangle under p is closer than anything provably visible and partly
    // visible from p: the query returns the lower bound, the hidden foot at 1.25 (how 3), and
    // counts it (assumption 2 of VisibleField.hpp).
    const Grid g(
        {-3, -1, -0.5, 0, 0.25, 0.5, 1, 3},
        {0, 1, 2},
        {0, 1, 2},
        [](const Eigen::Vector3d& c) {
            if (c[0] < -1 || c[0] > 1) return Kind::Input;
            if (c[0] > -0.5 && c[0] < 0 && c[1] < 1 && c[2] < 1) return Kind::Other;
            if (c[0] < 0.25) return Kind::Band;
            if (c[0] < 0.5) return Kind::Other;
            return Kind::Band;
        });
    const VisibleField vf = walls(-1, 1, 1.35);
    const auto f = g.front_face(0.25, 0, 1, 0, 1);
    REQUIRE(f[0] >= 0);
    const Eigen::Vector3d p = (g.P[size_t(f[0])] + g.P[size_t(f[1])] + g.P[size_t(f[2])]) / 3.;
    const auto r = vf.nearest(p, g.face_start(f), g);
    REQUIRE(r.status == VisibleField::Feature::Status::Found);
    CHECK(r.how == 3);
    CHECK(r.d == Catch::Approx(1.25));
    CHECK(r.foot[0] == Catch::Approx(-1.));
    CHECK(vf.counts().hidden_further == 1);
    CHECK(vf.counts().stops == 0);
}

TEST_CASE("visible-field-derivatives", "[offset][3d][visible]")
{
    // gradient and hessian against central differences of relative_residual with the feature
    // held, for a vertex, an edge and a face foot.
    const VisibleField vf = walls(-1, 1, 1.35);
    const Eigen::Vector3d p(0.3, 0.7, 0.9);
    for (int dim : {0, 1, 2}) {
        VisibleField::Feature f;
        f.status = VisibleField::Feature::Status::Found;
        f.dim = dim;
        f.dir = Eigen::Vector3d(0, 1, 0);
        f.foot = dim == 0
                     ? Eigen::Vector3d(-1, 0, 0)
                     : (dim == 1 ? Eigen::Vector3d(-1, 0.7, 0) : Eigen::Vector3d(-1, 0.7, 0.9));
        // Hold the feature: a vertex foot stays put, an edge foot slides along the edge, a face
        // foot along the face.
        const auto foot_at = [&](const Eigen::Vector3d& x) {
            VisibleField::Feature g = f;
            if (dim == 1) g.foot = Eigen::Vector3d(-1, x[1], 0);
            if (dim == 2) g.foot = Eigen::Vector3d(-1, x[1], x[2]);
            return g;
        };
        const double h = 1e-6;
        const Eigen::Vector3d gr = vf.gradient(p, f);
        const Eigen::Matrix3d H = vf.hessian(p, f);
        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3d e = Eigen::Vector3d::Zero();
            e[i] = h;
            const double fd = (vf.relative_residual(p + e, foot_at(p + e)) -
                               vf.relative_residual(p - e, foot_at(p - e))) /
                              (2 * h);
            INFO("dim " << dim << " i " << i);
            CHECK(gr[i] == Catch::Approx(fd).margin(1e-7));
            const Eigen::Vector3d gd =
                (vf.gradient(p + e, foot_at(p + e)) - vf.gradient(p - e, foot_at(p - e))) / (2 * h);
            for (int k = 0; k < 3; ++k) CHECK(H(k, i) == Catch::Approx(gd[k]).margin(1e-5));
        }
    }
}

TEST_CASE("visible-field-walk-fuzz", "[offset][3d][visible]")
{
    // The exact walk against a reference built from each cell's parameter interval on the
    // segment (the segment clipped by the cell's four half-spaces, in floating point), on random
    // grids of band, outside and input boxes. From a band vertex to a random point: visible
    // exactly when [0, b] is covered by band intervals with b > 0 and [b, 1] by input intervals.
    // Random targets keep the segment off mesh edges except at its start, where the walk decides
    // through the start cones.
    std::mt19937 rng(20261007);
    std::uniform_real_distribution<double> U(0.0, 3.0);
    std::uniform_int_distribution<int> kindd(0, 9);
    int compared = 0, visible_count = 0;
    for (int grid = 0; grid < 20; ++grid) {
        std::vector<Kind> box_kind(27);
        for (auto& k : box_kind) {
            const int r = kindd(rng);
            k = r < 6 ? Kind::Band : (r < 9 ? Kind::Other : Kind::Input);
        }
        const Grid g({0, 1, 2, 3}, {0, 1, 2, 3}, {0, 1, 2, 3}, [&](const Eigen::Vector3d& c) {
            const int i = int(c[0]), j = int(c[1]), k = int(c[2]);
            return box_kind[size_t((i * 3 + j) * 3 + k)];
        });
        // One triangle bounding no solid, so visible() is the walk alone.
        const VisibleField vf({{0, 0, 0}, {1, 0, 0}, {0, 1, 0}}, {{0, 1, 2}}, {int8_t(0)}, 1.0);
        for (int trial = 0; trial < 60; ++trial) {
            // trial % 3 == 1: a band vertex. == 0: a random point inside a front face (one side
            // band, the other not), whose rounded position is off the face by rounding only.
            // == 2: a band vertex and a target in a grid plane through it, so the segment runs
            // along mesh faces and edges (the grazing case, decided on closed cells).
            VisibleStart st;
            Eigen::Vector3d p;
            if (trial % 3 != 0) {
                const int64_t v = int64_t(std::uniform_int_distribution<int>(0, 63)(rng));
                st = g.vertex_start(v);
                p = g.P[size_t(v)];
            } else {
                std::vector<std::array<int64_t, 3>> fronts;
                for (const auto& [f, cs] : g.face_cells)
                    if (cs.size() == 2 &&
                        ((g.K[size_t(cs[0])] == Kind::Band) != (g.K[size_t(cs[1])] == Kind::Band)))
                        fronts.push_back(f);
                if (fronts.empty()) continue;
                const auto f = fronts[size_t(
                    std::uniform_int_distribution<int>(0, int(fronts.size()) - 1)(rng))];
                std::uniform_real_distribution<double> W(0.05, 1.0);
                double a = W(rng), b = W(rng), c = W(rng);
                const double sum = a + b + c;
                p = (a / sum) * g.P[size_t(f[0])] + (b / sum) * g.P[size_t(f[1])] +
                    (c / sum) * g.P[size_t(f[2])];
                st = g.face_start(f, {{a, b, c}});
            }
            if (st.cells.empty()) continue;
            Eigen::Vector3d q(U(rng), U(rng), U(rng));
            if (trial % 3 == 2) q[size_t(trial % 9 / 3)] = p[size_t(trial % 9 / 3)];
            const bool walk = vf.visible(p, q, 0, st, g);
            // Every cell's interval [t0, t1] of the segment p + t (q - p), t in [0, 1].
            std::vector<std::pair<double, double>> band, input;
            for (int64_t c = 0; c < int64_t(g.T.size()); ++c) {
                const auto& t = g.T[size_t(c)];
                double t0 = 0., t1 = 1.;
                for (int j = 0; j < 4 && t0 <= t1; ++j) {
                    Eigen::Vector3d a, b, cc;
                    int n = 0;
                    std::array<Eigen::Vector3d, 3> f;
                    for (int i = 0; i < 4; ++i)
                        if (i != j) f[size_t(n++)] = g.P[size_t(t[size_t(i)])];
                    Eigen::Vector3d nrm = (f[1] - f[0]).cross(f[2] - f[0]);
                    if (nrm.dot(g.P[size_t(t[size_t(j)])] - f[0]) < 0) nrm = -nrm; // inward
                    const double s0 = nrm.dot(p - f[0]), ds = nrm.dot(q - p);
                    // inside: s0 + t ds >= 0
                    if (std::abs(ds) < 1e-300) {
                        if (s0 < 0) t1 = -1;
                    } else if (ds > 0) {
                        t0 = std::max(t0, -s0 / ds);
                    } else {
                        t1 = std::min(t1, -s0 / ds);
                    }
                }
                if (t1 - t0 > 1e-12) {
                    if (g.K[size_t(c)] == Kind::Band) band.push_back({t0, t1});
                    if (g.K[size_t(c)] == Kind::Input) input.push_back({t0, t1});
                }
            }
            const auto prefix = [](std::vector<std::pair<double, double>> iv) {
                std::sort(iv.begin(), iv.end());
                double reach = 0.;
                for (const auto& [a, b] : iv) {
                    if (a > reach + 1e-9) break;
                    reach = std::max(reach, b);
                }
                return reach;
            };
            const auto suffix = [](std::vector<std::pair<double, double>> iv) {
                std::sort(iv.begin(), iv.end(), [](auto x, auto y) { return x.second > y.second; });
                double reach = 1.;
                for (const auto& [a, b] : iv) {
                    if (b < reach - 1e-9) break;
                    reach = std::min(reach, a);
                }
                return reach;
            };
            const double bp = prefix(band);
            const double is = suffix(input);
            const bool brute = bp > 1e-9 && (bp >= 1 - 1e-9 || is <= bp + 1e-9);
            ++compared;
            visible_count += brute ? 1 : 0;
            INFO(
                "grid " << grid << " trial " << trial << " p " << p.transpose() << " q "
                        << q.transpose() << " band prefix " << bp << " input suffix " << is);
            CHECK(walk == brute);
        }
    }
    INFO("compared " << compared << ", visible " << visible_count);
    CHECK(compared > 500);
    CHECK(visible_count > 50);
}

TEST_CASE("visible-field-first-step", "[offset][3d][visible]")
{
    // The band is the slab -1 < x < 0, y < 1; above it (y > 1) outside. The input wall leans
    // over the band: its triangles run from (-1, 0) to (-0.5, 2) in (x, y). From a point p on the
    // band's top face (y = 1) the wall's nearest point is above y = 1, in a direction that leaves
    // the band at p. The answer is the wall's nearest point among the directions that enter the
    // band: on the wall at y = 1, where x = -0.75.
    const Grid g({-3, -1, -0.5, 0, 3}, {0, 1, 2}, {0, 1}, [](const Eigen::Vector3d& c) {
        if (c[0] < -1) return Kind::Input;
        if (c[0] < 0 && c[1] < 1) return Kind::Band;
        return Kind::Other;
    });
    std::vector<Eigen::Vector3d> V = {{-1, 0, -1}, {-0.5, 2, -1}, {-0.5, 2, 2}, {-1, 0, 2}};
    std::vector<Eigen::Vector3i> F = {{0, 1, 2}, {0, 2, 3}};
    std::vector<int8_t> inner;
    for (const auto& f : F) {
        inner.push_back(int8_t(
            wmtk::utils::predicates::orient3d(
                V[size_t(f[0])],
                V[size_t(f[1])],
                V[size_t(f[2])],
                Eigen::Vector3d(-3, 1, 0.5))));
    }
    const VisibleField vf(V, F, inner, 1.0);
    std::array<int64_t, 3> f{-1, -1, -1};
    for (const auto& [fv, cs] : g.face_cells) {
        if (cs.size() != 2 ||
            (g.K[size_t(cs[0])] == Kind::Band) == (g.K[size_t(cs[1])] == Kind::Band))
            continue;
        bool ok = true;
        for (int64_t v : fv) {
            const auto& x = g.P[size_t(v)];
            ok = ok && std::abs(x[1] - 1) < 1e-12 && x[0] >= -0.5 - 1e-12 && x[0] <= 1e-12;
        }
        if (ok) {
            f = fv;
            break;
        }
    }
    REQUIRE(f[0] >= 0);
    const Eigen::Vector3d p = (g.P[size_t(f[0])] + g.P[size_t(f[1])] + g.P[size_t(f[2])]) / 3.;
    const auto r = vf.nearest(p, g.face_start(f), g);
    REQUIRE(r.status == VisibleField::Feature::Status::Found);
    CHECK(r.how == 1);
    CHECK(r.foot[1] == Catch::Approx(1.0).margin(1e-12));
    CHECK(r.foot[0] == Catch::Approx(-0.75).margin(1e-12));
    CHECK(r.d == Catch::Approx(std::abs(p[0] + 0.75)).margin(1e-12));
    CHECK(vf.counts().first_step == 1);
}

TEST_CASE("visible-field-nothing-visible", "[offset][3d][visible]")
{
    // Band A has crossed the midline and wall A is not in the input: wall B is hidden behind the
    // outside layer, nothing is visible, and the euclidean nearest point (wall B) is taken.
    const Grid g(
        {-3, -1, -0.5, 0.25, 0.5, 1, 3},
        {0, 1, 2},
        {0, 1, 2},
        [](const Eigen::Vector3d& c) {
            if (c[0] < -1 || c[0] > 1) return Kind::Input;
            if (c[0] < 0.25) return Kind::Band;
            if (c[0] < 0.5) return Kind::Other;
            return Kind::Band;
        });
    std::vector<Eigen::Vector3d> V = {{1, 0, 0}, {1, 2, 0}, {1, 2, 2}, {1, 0, 2}};
    std::vector<Eigen::Vector3i> F = {{0, 1, 2}, {0, 2, 3}};
    std::vector<int8_t> inner;
    for (const auto& f : F) {
        inner.push_back(int8_t(
            wmtk::utils::predicates::orient3d(
                V[size_t(f[0])],
                V[size_t(f[1])],
                V[size_t(f[2])],
                Eigen::Vector3d(2, 1, 1))));
    }
    const VisibleField vf(V, F, inner, 1.35);
    const int64_t v = g.vertex_at({0.25, 1, 1});
    const auto r = vf.nearest(g.P[size_t(v)], g.vertex_start(v), g);
    REQUIRE(r.status == VisibleField::Feature::Status::Found);
    CHECK(r.how == 2);
    CHECK(r.d == Catch::Approx(0.75));
    CHECK(vf.counts().none_visible == 1);
}
