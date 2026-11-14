#include <wmtk/components/simwild/MaterialInterface.hpp>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>

#include <array>
#include <map>
#include <random>
#include <set>
#include <vector>

using namespace wmtk;
namespace mi = wmtk::components::simwild::material_interface;

namespace {

using R = Rational;

R det3(const std::array<R, 3>& a, const std::array<R, 3>& b, const std::array<R, 3>& c)
{
    return a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
           a[2] * (b[0] * c[1] - b[1] * c[0]);
}

std::array<R, 3> minus(const std::array<R, 3>& a, const std::array<R, 3>& b)
{
    return {{a[0] - b[0], a[1] - b[1], a[2] - b[2]}};
}

/// f at barycentric coordinates t, exactly
R eval(const std::array<double, 4>& f, const std::array<R, 4>& t)
{
    R s(0.);
    for (int c = 0; c < 4; ++c) {
        s = s + R(f[c]) * t[c];
    }
    return s;
}

/// the determinant of a small matrix of doubles, by the Leibniz expansion in rationals
R det_reference(const std::vector<double>& a, const int n)
{
    const auto& p = mi::exact::permutations(n);
    R det(0.);
    for (size_t t = 0; t < p.order.size(); ++t) {
        R product(double(p.sign[t]));
        for (int i = 0; i < n; ++i) {
            product = product * R(a[i * n + p.order[t][i]]);
        }
        det = det + product;
    }
    return det;
}

/// random function values: uniform doubles, or small integers, which make ties and planes through
/// vertices common
std::vector<std::array<double, 4>>
random_values(std::mt19937& rng, const size_t n, const bool integers)
{
    std::uniform_real_distribution<double> real(0., 1.);
    std::uniform_int_distribution<int> small(0, 3);
    std::vector<std::array<double, 4>> v(n);
    for (auto& f : v) {
        for (double& x : f) {
            x = integers ? double(small(rng)) : real(rng);
        }
    }
    return v;
}

/**
 * Certifies one tet's diagram exactly, in barycentric space (the tet's corners at the origin and
 * the
 * unit vectors, volume 1/6): each cell's boundary is closed and consistently oriented, each cell has
 * a positive volume and its function smallest (SENSE) at all its vertices, so in all of it, and the
 * volumes add up to the tet's, so the cells cover it without overlap.
 */
void certify_tet(const mi::TetPartition& tp, const mi::Sense sense)
{
    const auto& tv = tp.values;
    R total(0.);
    std::set<size_t> functions;
    for (const mi::Cell& cell : tp.cells) {
        const size_t m = size_t(cell.function - mi::first_function);
        CHECK(functions.insert(m).second);

        std::vector<std::array<R, 4>> t;
        for (const mi::Joint& j : cell.vertices) {
            t.push_back(mi::joint_barycentric_exact(tv, j));
        }
        for (const auto& tb : t) {
            const R fm = eval(tv.values[m], tb);
            for (size_t j = 0; j < tv.functions.size(); ++j) {
                const R fj = eval(tv.values[j], tb);
                CHECK((sense == mi::Sense::Min ? fm <= fj : fm >= fj));
            }
            for (int c = 0; c < 4; ++c) {
                CHECK(tb[c].get_sign() >= 0);
            }
        }

        // closed and oriented: each directed edge once, and its reverse once
        std::map<std::pair<uint32_t, uint32_t>, int> edges;
        for (const auto& face : cell.faces) {
            const auto& vs = face.vertices;
            REQUIRE(vs.size() >= 3);
            for (size_t i = 0; i < vs.size(); ++i) {
                ++edges[{vs[i], vs[(i + 1) % vs.size()]}];
            }
        }
        for (const auto& [e, count] : edges) {
            CHECK(count == 1);
            CHECK(edges.count({e.second, e.first}) == 1);
        }

        // volume: coned from vertex 0
        const auto point = [&](const uint32_t v) {
            return std::array<R, 3>{{t[v][1], t[v][2], t[v][3]}};
        };
        R volume(0.);
        for (const auto& face : cell.faces) {
            const auto& vs = face.vertices;
            if (std::find(vs.begin(), vs.end(), 0u) != vs.end()) {
                continue;
            }
            for (size_t i = 1; i + 1 < vs.size(); ++i) {
                volume = volume + det3(
                                      minus(point(vs[0]), point(0)),
                                      minus(point(vs[i]), point(0)),
                                      minus(point(vs[i + 1]), point(0)));
            }
        }
        CHECK(volume.get_sign() > 0);
        total = total + volume;
    }
    CHECK(total == R(1.)); // 6 x the volume 1/6
}

/// the cube [0, n]^3 of unit cubes, each cut into 6 positively oriented tets (Kuhn)
void kuhn_grid(const int n, std::vector<Vector3d>& points, std::vector<std::array<size_t, 4>>& tets)
{
    const auto id = [n](int i, int j, int k) { return size_t((i * (n + 1) + j) * (n + 1) + k); };
    points.clear();
    for (int i = 0; i <= n; ++i) {
        for (int j = 0; j <= n; ++j) {
            for (int k = 0; k <= n; ++k) {
                points.emplace_back(i, j, k);
            }
        }
    }
    tets.clear();
    std::array<int, 3> axes{{0, 1, 2}};
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            for (int k = 0; k < n; ++k) {
                std::sort(axes.begin(), axes.end());
                do {
                    std::array<int, 3> c{{i, j, k}};
                    std::array<size_t, 4> t{};
                    t[0] = id(c[0], c[1], c[2]);
                    for (int s = 0; s < 3; ++s) {
                        ++c[axes[s]];
                        t[s + 1] = id(c[0], c[1], c[2]);
                    }
                    const Vector3d& p0 = points[t[0]];
                    if ((points[t[1]] - p0).dot((points[t[2]] - p0).cross(points[t[3]] - p0)) < 0) {
                        std::swap(t[2], t[3]);
                    }
                    tets.push_back(t);
                } while (std::next_permutation(axes.begin(), axes.end()));
            }
        }
    }
}

} // namespace

TEST_CASE("material_interface_det_sign", "[simwild][material_interface]")
{
    std::mt19937 rng(7);
    std::uniform_int_distribution<int> small(-2, 2);
    std::uniform_real_distribution<double> real(-1., 1.);
    for (int n = 1; n <= 5; ++n) {
        for (int trial = 0; trial < 300; ++trial) {
            std::vector<double> a(n * n);
            for (double& x : a) {
                x = small(rng);
            }
            if (trial % 3 == 1) {
                // rows that nearly cancel: the filter cannot decide
                for (int c = 0; c < n && n > 1; ++c) {
                    a[(n - 1) * n + c] = a[c] + std::ldexp(real(rng), -50);
                }
            } else if (trial % 3 == 2) {
                for (double& x : a) {
                    x = real(rng);
                }
            }
            CHECK(mi::exact::det_sign(a.data(), n) == det_reference(a, n).get_sign());
        }
    }
}

TEST_CASE("material_interface_joint_predicates", "[simwild][material_interface]")
{
    // every joint shape: at a corner (1 function), on an edge (2), on a face (3), inside (4)
    std::mt19937 rng(11);
    int checked = 0;
    for (int trial = 0; trial < 400; ++trial) {
        mi::TetValues tv;
        tv.values = random_values(rng, 5, trial % 2 == 0);
        for (size_t f = 0; f < 5; ++f) {
            tv.functions.push_back(f);
        }
        for (int n = 1; n <= 4; ++n) {
            // the boundary planes: the first 4 - n corners' opposite faces, rotated by the trial
            mi::Joint joint{};
            for (int b = 0; b < 4 - n; ++b) {
                joint[b] = (b + trial) % 4;
            }
            for (int i = 0; i < n; ++i) {
                joint[4 - n + i] = mi::first_function + i;
            }
            std::sort(joint.begin(), joint.end());
            // a joint is a point only if det A is not 0
            const mi::JointParts p = mi::joint_parts(joint);
            std::vector<double> a((n + 1) * (n + 1));
            std::array<double, 5> last{};
            for (int c = 1; c <= n; ++c) {
                last[c] = 1.;
            }
            mi::joint_matrix(tv, p, last.data(), a.data());
            if (det_reference(a, n + 1).get_sign() == 0) {
                continue;
            }
            const auto t = mi::joint_barycentric_exact(tv, joint);
            const R f = eval(tv.values[p.functions[0]], t);
            for (int i = 1; i < n; ++i) {
                CHECK(eval(tv.values[p.functions[i]], t) == f); // the functions meet there
            }
            const auto g = tv.values[4];
            CHECK(mi::joint_sign(tv, joint, g) == (eval(g, t) - f).get_sign());
            for (int c = 0; c < 4; ++c) {
                CHECK(mi::barycentric_sign(tv, joint, c) == t[c].get_sign());
            }
            const auto td = mi::joint_barycentric(tv, joint);
            for (int c = 0; c < 4; ++c) {
                CHECK(std::abs(td[c] - t[c].to_double()) < 1e-9 * (1 + std::abs(t[c].to_double())));
            }
            ++checked;
        }
    }
    CHECK(checked > 800);
}

TEST_CASE("material_interface_one_tet", "[simwild][material_interface]")
{
    std::mt19937 rng(3);
    for (const mi::Sense sense : {mi::Sense::Min, mi::Sense::Max}) {
        for (int trial = 0; trial < 600; ++trial) {
            const size_t n = 1 + trial % 6;
            const bool integers = trial % 2 == 1;
            const auto values = random_values(rng, n, integers);
            mi::TetValues tv;
            for (const size_t i : mi::active_functions(values, sense)) {
                tv.functions.push_back(i);
                tv.values.push_back(values[i]);
            }
            const mi::TetPartition tp = mi::partition_tet(tv, sense);
            certify_tet(tp, sense);

            // the functions active_functions drops change nothing
            mi::TetValues all;
            for (size_t i = 0; i < n; ++i) {
                all.functions.push_back(i);
                all.values.push_back(values[i]);
            }
            const mi::TetPartition tp_all = mi::partition_tet(all, sense);
            std::set<size_t> a, b;
            for (const auto& c : tp.cells) {
                a.insert(tv.functions[c.function - mi::first_function]);
            }
            for (const auto& c : tp_all.cells) {
                b.insert(all.functions[c.function - mi::first_function]);
            }
            CHECK(a == b);
            certify_tet(tp_all, sense);
        }
    }
}

TEST_CASE("material_interface_sense", "[simwild][material_interface]")
{
    // SENSE: the min diagram of f is the max diagram of -f
    std::mt19937 rng(5);
    for (int trial = 0; trial < 200; ++trial) {
        const auto values = random_values(rng, 1 + trial % 5, trial % 2 == 1);
        auto negated = values;
        for (auto& f : negated) {
            for (double& x : f) {
                x = -x;
            }
        }
        mi::TetValues a, b;
        for (const size_t i : mi::active_functions(values, mi::Sense::Min)) {
            a.functions.push_back(i);
            a.values.push_back(values[i]);
        }
        for (const size_t i : mi::active_functions(negated, mi::Sense::Max)) {
            b.functions.push_back(i);
            b.values.push_back(negated[i]);
        }
        REQUIRE(a.functions == b.functions);
        const auto pa = mi::partition_tet(a, mi::Sense::Min);
        const auto pb = mi::partition_tet(b, mi::Sense::Max);
        REQUIRE(pa.cells.size() == pb.cells.size());
        for (size_t c = 0; c < pa.cells.size(); ++c) {
            CHECK(pa.cells[c].function == pb.cells[c].function);
            CHECK(pa.cells[c].vertices == pb.cells[c].vertices);
        }
    }
}

TEST_CASE("material_interface_mesh", "[simwild][material_interface]")
{
    std::vector<Vector3d> points;
    std::vector<std::array<size_t, 4>> tets;
    const int cubes = 3;
    kuhn_grid(cubes, points, tets);

    std::mt19937 rng(17);
    std::uniform_real_distribution<double> real(-0.5, cubes + 0.5);
    std::uniform_int_distribution<int> small(0, 3);
    for (int trial = 0; trial < 24; ++trial) {
        // distances to random points (smooth, generic), or small integers (ties everywhere)
        const size_t n_functions = 2 + trial % 4;
        const bool integers = trial % 3 == 2;
        std::vector<Vector3d> sites(n_functions);
        for (auto& s : sites) {
            s = Vector3d(real(rng), real(rng), real(rng));
        }
        std::vector<std::vector<double>> table(points.size(), std::vector<double>(n_functions));
        for (size_t v = 0; v < points.size(); ++v) {
            for (size_t f = 0; f < n_functions; ++f) {
                table[v][f] = integers ? double(small(rng)) : (points[v] - sites[f]).norm();
            }
        }
        const std::function<double(size_t, size_t)> value = [&](size_t v, size_t f) {
            return table[v][f];
        };
        const mi::MeshPartition mp = mi::partition_mesh(tets, n_functions, value);

        // exact positions
        std::vector<std::array<R, 3>> exact(mp.vertices.size());
        for (size_t v = 0; v < mp.vertices.size(); ++v) {
            const auto t = mi::exact_barycentric(mp.vertices[v], tets, value);
            for (int k = 0; k < 3; ++k) {
                R x(0.);
                for (int c = 0; c < 4; ++c) {
                    x = x + t[c] * R(points[tets[mp.vertices[v].tet][c]][k]);
                }
                exact[v][k] = x;
            }
        }
        // no two vertices at one point
        std::set<std::array<R, 3>> seen;
        for (const auto& p : exact) {
            CHECK(seen.insert(p).second);
        }

        // the labelled tets: positive volumes, adding up to the cube's, conforming
        const auto lts = mi::tetrahedralize(mp);
        R total(0.);
        std::map<std::array<size_t, 3>, std::vector<int>> triangles; // sorted -> orientations
        for (const auto& lt : lts) {
            const auto& v = lt.vertices;
            const R volume = det3(
                minus(exact[v[1]], exact[v[0]]),
                minus(exact[v[2]], exact[v[0]]),
                minus(exact[v[3]], exact[v[0]]));
            CHECK(volume.get_sign() > 0);
            total = total + volume;
            for (int skip = 0; skip < 4; ++skip) {
                std::array<size_t, 3> tri{};
                int k = 0;
                for (int c = 0; c < 4; ++c) {
                    if (c != skip) {
                        tri[k++] = v[c];
                    }
                }
                // the orientation of tri as a face of this tet: the parity of the sort
                int parity = (skip % 2 == 0) ? 1 : -1;
                for (int i = 0; i < 3; ++i) {
                    for (int j = i + 1; j < 3; ++j) {
                        if (tri[i] > tri[j]) {
                            parity = -parity;
                        }
                    }
                }
                std::sort(tri.begin(), tri.end());
                triangles[tri].push_back(parity);
            }

            // the label: smallest at the centroid, exactly, in the mesh tet
            std::array<R, 3> centroid;
            for (int k = 0; k < 3; ++k) {
                centroid[k] =
                    (exact[v[0]][k] + exact[v[1]][k] + exact[v[2]][k] + exact[v[3]][k]) / R(4.);
            }
            const auto& mt = tets[lt.tet];
            std::array<std::array<R, 3>, 4> corner;
            for (int c = 0; c < 4; ++c) {
                for (int k = 0; k < 3; ++k) {
                    corner[c][k] = R(points[mt[c]][k]);
                }
            }
            const R whole = det3(
                minus(corner[1], corner[0]),
                minus(corner[2], corner[0]),
                minus(corner[3], corner[0]));
            std::array<R, 4> t;
            t[1] = det3(
                       minus(centroid, corner[0]),
                       minus(corner[2], corner[0]),
                       minus(corner[3], corner[0])) /
                   whole;
            t[2] = det3(
                       minus(corner[1], corner[0]),
                       minus(centroid, corner[0]),
                       minus(corner[3], corner[0])) /
                   whole;
            t[3] = det3(
                       minus(corner[1], corner[0]),
                       minus(corner[2], corner[0]),
                       minus(centroid, corner[0])) /
                   whole;
            t[0] = R(1.) - t[1] - t[2] - t[3];
            for (int c = 0; c < 4; ++c) {
                CHECK(t[c].get_sign() >= 0);
            }
            const auto f_at = [&](size_t f) {
                return eval(
                    {{table[mt[0]][f], table[mt[1]][f], table[mt[2]][f], table[mt[3]][f]}},
                    t);
            };
            const R own = f_at(lt.function);
            for (size_t f = 0; f < n_functions; ++f) {
                CHECK(own <= f_at(f));
            }
        }
        CHECK(total == R(6. * cubes * cubes * cubes)); // 6 x the cube's volume

        for (const auto& [tri, orientations] : triangles) {
            bool on_boundary = false;
            for (int k = 0; k < 3; ++k) {
                for (const double side : {0., double(cubes)}) {
                    bool all = true;
                    for (const size_t v : tri) {
                        all = all && exact[v][k] == R(side);
                    }
                    on_boundary = on_boundary || all;
                }
            }
            if (on_boundary) {
                CHECK(orientations.size() == 1);
            } else {
                REQUIRE(orientations.size() == 2);
                CHECK(orientations[0] == -orientations[1]);
            }
        }
    }
}
