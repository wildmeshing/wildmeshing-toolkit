#include "VisibleField.hpp"

#include <wmtk/Types.hpp>
#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/predicates.hpp>

#include <algorithm>
#include <limits>
#include <numeric>
#include <queue>
#include <unordered_set>

namespace wmtk::components::topological_offset {

namespace {

using Vec3 = Eigen::Vector3d;

/// The exact sign of det[b - a, c - a, d - a]: +1, 0 or -1.
int o3(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d)
{
    return int(wmtk::utils::predicates::orient3d(a, b, c, d));
}

/// Local vertices of the face opposite local vertex j.
constexpr std::array<std::array<int, 3>, 4> kFace = {{{1, 2, 3}, {0, 2, 3}, {0, 1, 3}, {0, 1, 2}}};

/// A band start cell as the queries use it: its corners, which faces pass through p, and for
/// each face the sign of its own opposite vertex (the cell's side).
struct Cone
{
    int64_t cell = -1;
    std::array<int64_t, 4> v;
    std::array<Vec3, 4> X;
    std::array<int, 4> s_in;
    uint8_t mask = 0;
};

std::vector<Cone> start_cones(const VisibleStart& start, const VisibilityCells& cells)
{
    std::vector<Cone> out;
    for (const auto& [cell, mask] : start.cells) {
        if (cells.kind(cell) != VisibilityCells::Kind::Band) continue;
        Cone c;
        c.cell = cell;
        c.mask = mask;
        c.v = cells.vertices(cell);
        for (int i = 0; i < 4; ++i) c.X[size_t(i)] = cells.position(c.v[size_t(i)]);
        for (int j = 0; j < 4; ++j) {
            const auto& f = kFace[size_t(j)];
            c.s_in[size_t(j)] =
                o3(c.X[size_t(f[0])], c.X[size_t(f[1])], c.X[size_t(f[2])], c.X[size_t(j)]);
        }
        out.push_back(c);
    }
    return out;
}

/// True when every point lies strictly outside, for each cone, one of its faces through p: no
/// segment from p that starts into the band can reach any of them. Exact; a sufficient test.
template <typename Points>
bool cone_hidden(const std::vector<Cone>& cones, const Points& pts)
{
    for (const Cone& c : cones) {
        bool separated = false;
        for (int j = 0; j < 4 && !separated; ++j) {
            if (!(c.mask & (1u << j))) continue;
            const auto& f = kFace[size_t(j)];
            bool all_out = true;
            for (const Vec3& x : pts) {
                if (o3(c.X[size_t(f[0])], c.X[size_t(f[1])], c.X[size_t(f[2])], x) *
                        c.s_in[size_t(j)] >=
                    0) {
                    all_out = false;
                    break;
                }
            }
            separated = all_out;
        }
        if (!separated) return false;
    }
    return true;
}

/// det[b - a, c - a, d - a] in exact rationals.
wmtk::Rational det_r(
    const wmtk::Vector3r& a,
    const wmtk::Vector3r& b,
    const wmtk::Vector3r& c,
    const wmtk::Vector3r& d)
{
    const wmtk::Vector3r u = b - a, v = c - a, w = d - a;
    return u[0] * (v[1] * w[2] - v[2] * w[1]) - u[1] * (v[0] * w[2] - v[2] * w[0]) +
           u[2] * (v[0] * w[1] - v[1] * w[0]);
}

/// Triangle (a, b, c) clipped, in exact rationals, by one start cell's closed half-spaces
/// through p: the part of it a segment from p can reach after starting into that cell.
std::vector<wmtk::Vector3r>
clip_by_cone(const Cone& cn, const Vec3& a, const Vec3& b, const Vec3& c)
{
    std::vector<wmtk::Vector3r> poly = {
        wmtk::to_rational(a),
        wmtk::to_rational(b),
        wmtk::to_rational(c)};
    for (int j = 0; j < 4 && !poly.empty(); ++j) {
        if (!(cn.mask & (1u << j))) continue;
        const auto& f = kFace[size_t(j)];
        const wmtk::Vector3r A = wmtk::to_rational(cn.X[size_t(f[0])]);
        const wmtk::Vector3r B = wmtk::to_rational(cn.X[size_t(f[1])]);
        const wmtk::Vector3r C = wmtk::to_rational(cn.X[size_t(f[2])]);
        // The cell's side, in det_r's own sign convention (orient3d's is the opposite one).
        const int s = det_r(A, B, C, wmtk::to_rational(cn.X[size_t(j)])).get_sign();
        std::vector<wmtk::Rational> val(poly.size());
        for (size_t i = 0; i < poly.size(); ++i) {
            val[i] = det_r(A, B, C, poly[i]);
            if (s < 0) val[i] = -val[i];
        }
        std::vector<wmtk::Vector3r> next;
        for (size_t i = 0; i < poly.size(); ++i) {
            const size_t k = (i + 1) % poly.size();
            const int si = val[i].get_sign(), sk = val[k].get_sign();
            if (si >= 0) next.push_back(poly[i]);
            if ((si > 0 && sk < 0) || (si < 0 && sk > 0)) {
                const wmtk::Rational t = val[i] / (val[i] - val[k]);
                next.push_back(poly[i] + (poly[k] - poly[i]) * t);
            }
        }
        poly = std::move(next);
    }
    return poly;
}

/// Closest point of triangle (a, b, c) to p, and its feature: 2 interior, 1 edge (dir its unit
/// direction), 0 vertex. Ericson, Real-Time Collision Detection, 5.1.5.
Vec3 closest_on_triangle(
    const Vec3& p,
    const Vec3& a,
    const Vec3& b,
    const Vec3& c,
    int& dim,
    Vec3& dir)
{
    dir.setZero();
    const Vec3 ab = b - a, ac = c - a, ap = p - a;
    const double d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0 && d2 <= 0) {
        dim = 0;
        return a;
    }
    const Vec3 bp = p - b;
    const double d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0 && d4 <= d3) {
        dim = 0;
        return b;
    }
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        dim = 1;
        dir = ab.normalized();
        return a + (d1 / (d1 - d3)) * ab;
    }
    const Vec3 cp = p - c;
    const double d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0 && d5 <= d6) {
        dim = 0;
        return c;
    }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        dim = 1;
        dir = ac.normalized();
        return a + (d2 / (d2 - d6)) * ac;
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        dim = 1;
        dir = (c - b).normalized();
        return b + ((d4 - d3) / ((d4 - d3) + (d5 - d6))) * (c - b);
    }
    const double denom = 1. / (va + vb + vc);
    dim = 2;
    return a + ab * (vb * denom) + ac * (vc * denom);
}

double box_dist2(const Vec3& p, const Vec3& lo, const Vec3& hi)
{
    double s = 0.;
    for (int i = 0; i < 3; ++i) {
        const double e = std::max({lo[i] - p[i], 0., p[i] - hi[i]});
        s += e * e;
    }
    return s;
}

enum class Walk { Visible, Hidden, Inconsistent };

/// The walk's orientation predicate, on doubles and on exact rationals, in one sign convention:
/// orient3d's (Shewchuk's det[a - d; b - d; c - d]), which is minus det_r's. So the double cones'
/// signs hold in a rational walk too.
int sgn3(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d)
{
    return o3(a, b, c, d);
}
int sgn3(
    const wmtk::Vector3r& a,
    const wmtk::Vector3r& b,
    const wmtk::Vector3r& c,
    const wmtk::Vector3r& d)
{
    return -det_r(a, b, c, d).get_sign();
}
template <typename Pt>
Pt as_pt(const Vec3& x);
template <>
Vec3 as_pt<Vec3>(const Vec3& x)
{
    return x;
}
template <>
wmtk::Vector3r as_pt<wmtk::Vector3r>(const Vec3& x)
{
    return wmtk::to_rational(x);
}

/// The walk from p to q (see VisibleField::visible()), from the start cones; on doubles, or on
/// exact rationals for a q that is not a double (a point of a clipped triangle).
template <typename Pt>
Walk walk_t(
    const Pt& p,
    const Pt& q,
    const std::vector<Cone>& cones,
    const VisibilityCells& cells,
    const int forced = -1)
{
    using Kind = VisibilityCells::Kind;
    // The start cell: a band cell of p whose closed cone at p contains the direction to q, i.e.
    // q on the closed inner side of each of its faces through p. Any one will do: the segment's
    // first stretch lies in each such cell's closure. `forced`: the cone q was clipped into, so
    // q is in it by construction (its rounded coordinates may sit a hair outside).
    int64_t cell = -1;
    uint8_t start_mask = 0;
    if (forced >= 0) {
        cell = cones[size_t(forced)].cell;
        start_mask = cones[size_t(forced)].mask;
    }
    for (const Cone& c : cones) {
        if (cell >= 0) break;
        bool in = true;
        for (int j = 0; j < 4 && in; ++j) {
            if (!(c.mask & (1u << j))) continue;
            const auto& f = kFace[size_t(j)];
            if (sgn3(
                    as_pt<Pt>(c.X[size_t(f[0])]),
                    as_pt<Pt>(c.X[size_t(f[1])]),
                    as_pt<Pt>(c.X[size_t(f[2])]),
                    q) *
                    c.s_in[size_t(j)] <
                0) {
                in = false;
            }
        }
        if (in) {
            cell = c.cell;
            start_mask = c.mask;
            break;
        }
    }
    if (cell < 0) return Walk::Hidden;

    bool in_input = false; // the walk may end in input cells, never leave them
    int64_t prev = -1;
    std::unordered_set<int64_t> seen;
    std::vector<int64_t> around;
    for (size_t step = 0; step < 1000000; ++step) {
        if (!seen.insert(cell).second) return Walk::Inconsistent;
        const auto v = cells.vertices(cell);
        std::array<Pt, 4> X;
        for (int i = 0; i < 4; ++i) X[size_t(i)] = as_pt<Pt>(cells.position(v[size_t(i)]));
        std::array<int, 4> s_in, s_q;
        bool q_in = true;
        for (int j = 0; j < 4; ++j) {
            const auto& f = kFace[size_t(j)];
            s_in[size_t(j)] = sgn3(X[size_t(f[0])], X[size_t(f[1])], X[size_t(f[2])], X[size_t(j)]);
            s_q[size_t(j)] = sgn3(X[size_t(f[0])], X[size_t(f[1])], X[size_t(f[2])], q);
            if (s_q[size_t(j)] * s_in[size_t(j)] < 0) q_in = false;
        }
        if (q_in) return Walk::Visible; // q in this closed cell, which is band or (ending) input

        // Leave the cell: the face whose plane q is strictly beyond and whose closed triangle the
        // line pq passes through. The three edge orientations of the line against the triangle
        // agree in sign exactly when it does; zeros put the crossing on an edge or a vertex.
        int exit_face = -1;
        int64_t deg_a = -1, deg_b = -1; // degenerate exit: an edge (a, b) or a vertex (a, -1)
        for (int j = 0; j < 4; ++j) {
            if (s_q[size_t(j)] * s_in[size_t(j)] >= 0) continue;
            if (prev < 0 && (start_mask & (1u << j))) continue; // a face through p
            const auto& f = kFace[size_t(j)];
            const int ia = f[0], ib = f[1], ic = f[2];
            // The segment crosses this face's plane between p and q only with p on the cell's
            // side of it; the line test below is about the infinite line, which also meets
            // planes behind p.
            if (sgn3(X[size_t(ia)], X[size_t(ib)], X[size_t(ic)], p) * s_in[size_t(j)] < 0)
                continue;
            const int oab = sgn3(p, q, X[size_t(ia)], X[size_t(ib)]);
            const int obc = sgn3(p, q, X[size_t(ib)], X[size_t(ic)]);
            const int oca = sgn3(p, q, X[size_t(ic)], X[size_t(ia)]);
            const bool pos = oab > 0 || obc > 0 || oca > 0, neg = oab < 0 || obc < 0 || oca < 0;
            if (pos && neg) continue;
            const int zeros = int(oab == 0) + int(obc == 0) + int(oca == 0);
            if (zeros == 0) {
                exit_face = j;
                break;
            }
            if (deg_a >= 0) continue; // already have the degenerate element
            if (zeros == 1) {
                if (oab == 0) {
                    deg_a = v[size_t(ia)];
                    deg_b = v[size_t(ib)];
                } else if (obc == 0) {
                    deg_a = v[size_t(ib)];
                    deg_b = v[size_t(ic)];
                } else {
                    deg_a = v[size_t(ic)];
                    deg_b = v[size_t(ia)];
                }
            } else if (zeros == 2) {
                deg_a = oab != 0 ? v[size_t(ic)] : (obc != 0 ? v[size_t(ia)] : v[size_t(ib)]);
                deg_b = -1;
            }
        }

        // The next cell, and which ones may continue the segment.
        int64_t next = -1;
        if (exit_face >= 0) {
            next = cells.neighbor(cell, exit_face);
        } else if (deg_a >= 0) {
            // Through an edge or a vertex: every cell around it whose closed wedge (edge) or
            // cone (vertex) contains the direction to q holds the segment's next stretch in its
            // closure. A band cell is preferred, then (only from band cells or input cells) an
            // input cell; the segment is covered if any of them qualifies.
            around.clear();
            if (deg_b >= 0) {
                cells.cells_around_edge(deg_a, deg_b, around);
            } else {
                cells.cells_around_vertex(deg_a, around);
            }
            int64_t best_band = -1, best_input = -1;
            for (const int64_t t : around) {
                if (t == cell || t == prev) continue;
                const auto tv = cells.vertices(t);
                std::array<Pt, 4> TX;
                for (int i = 0; i < 4; ++i)
                    TX[size_t(i)] = as_pt<Pt>(cells.position(tv[size_t(i)]));
                bool contains = true;
                for (int j = 0; j < 4 && contains; ++j) {
                    // Only the faces through the edge or vertex bound the wedge or cone.
                    // A face contains the edge (vertex) when its opposite vertex is not on it.
                    const auto& f = kFace[size_t(j)];
                    const int64_t opp = tv[size_t(j)];
                    const bool through =
                        deg_b >= 0 ? (opp != deg_a && opp != deg_b) : (opp != deg_a);
                    if (!through) continue;
                    const int si =
                        sgn3(TX[size_t(f[0])], TX[size_t(f[1])], TX[size_t(f[2])], TX[size_t(j)]);
                    const int sq = sgn3(TX[size_t(f[0])], TX[size_t(f[1])], TX[size_t(f[2])], q);
                    if (sq * si < 0) contains = false;
                }
                if (!contains) continue;
                const Kind k = cells.kind(t);
                if (k == Kind::Band && best_band < 0) best_band = t;
                if (k == Kind::Input && best_input < 0) best_input = t;
            }
            if (!in_input && best_band >= 0) {
                next = best_band;
            } else if (best_input >= 0) {
                next = best_input;
            } else {
                return Walk::Hidden;
            }
        } else {
            // q is beyond the cell but the line meets none of its faces: only possible when p's
            // rounded position is off its support in a way the walk cannot resolve.
            return Walk::Inconsistent;
        }
        if (next < 0) return Walk::Hidden; // the domain's boundary
        const Kind k = cells.kind(next);
        if (in_input) {
            if (k != Kind::Input) return Walk::Hidden; // the segment left the input again
        } else if (k == Kind::Input) {
            in_input = true;
        } else if (k != Kind::Band) {
            return Walk::Hidden;
        }
        prev = cell;
        cell = next;
    }
    return Walk::Inconsistent;
}

Walk walk(
    const Vec3& p,
    const Vec3& q,
    const std::vector<Cone>& cones,
    const VisibilityCells& cells,
    const int forced = -1)
{
    return walk_t<Vec3>(p, q, cones, cells, forced);
}

/// The point of a convex polygon (exact rational vertices, in order) nearest to p, exactly: the
/// projection of p onto its plane when inside, else the nearest point of its boundary. Rational
/// throughout -- projections and clamped segment parameters need no square root.
wmtk::Vector3r nearest_on_polygon_r(
    const wmtk::Vector3r& p,
    const std::vector<wmtk::Vector3r>& poly)
{
    using R = wmtk::Rational;
    const auto on_segment = [&](const wmtk::Vector3r& a, const wmtk::Vector3r& b) {
        const wmtk::Vector3r ab = b - a;
        const R l2 = ab.dot(ab);
        if (l2.get_sign() == 0) return a;
        R t = (p - a).dot(ab) / l2;
        if (t.get_sign() < 0) t = R(0);
        if ((t - R(1)).get_sign() > 0) t = R(1);
        return wmtk::Vector3r(a + ab * t);
    };
    if (poly.size() == 1) return poly[0];
    if (poly.size() == 2) return on_segment(poly[0], poly[1]);
    // The normal from the first triple that spans the plane; the clip keeps the triangle's
    // winding, so it orients every edge test alike.
    wmtk::Vector3r n(R(0), R(0), R(0));
    for (size_t i = 1; i + 1 < poly.size(); ++i) {
        const wmtk::Vector3r c = (poly[i] - poly[0]).cross(poly[i + 1] - poly[0]);
        if (c.dot(c).get_sign() != 0) {
            n = c;
            break;
        }
    }
    if (n.dot(n).get_sign() != 0) {
        const wmtk::Vector3r proj = p - n * ((n.dot(p - poly[0])) / n.dot(n));
        bool inside = true;
        for (size_t i = 0; i < poly.size() && inside; ++i) {
            const wmtk::Vector3r& a = poly[i];
            const wmtk::Vector3r& b = poly[(i + 1) % poly.size()];
            inside = (b - a).cross(proj - a).dot(n).get_sign() >= 0;
        }
        if (inside) return proj;
    }
    wmtk::Vector3r best = poly[0];
    bool have = false;
    R bd(0);
    for (size_t i = 0; i < poly.size(); ++i) {
        const wmtk::Vector3r q = on_segment(poly[i], poly[(i + 1) % poly.size()]);
        const R d = (q - p).dot(q - p);
        if (!have || (d - bd).get_sign() < 0) {
            bd = d;
            best = q;
            have = true;
        }
    }
    return best;
}

/// The query point exactly on its support (VisibleStart::support), or its rounded coordinates.
wmtk::Vector3r exact_point(const Vec3& p, const VisibleStart& start, const VisibilityCells& cells)
{
    if (start.support.empty()) return wmtk::to_rational(p);
    wmtk::Rational sum(0.);
    wmtk::Vector3r acc(wmtk::Rational(0.), wmtk::Rational(0.), wmtk::Rational(0.));
    for (const auto& [v, w] : start.support) {
        const wmtk::Rational rw(w);
        acc = acc + wmtk::to_rational(cells.position(v)) * rw;
        sum = sum + rw;
    }
    return acc / sum;
}

} // namespace

VisibleField::VisibleField(
    std::vector<Eigen::Vector3d> V,
    std::vector<Eigen::Vector3i> F,
    std::vector<int8_t> inner_sign,
    const double delta)
    : m_V(std::move(V))
    , m_F(std::move(F))
    , m_inner(std::move(inner_sign))
    , m_delta(delta)
{
    if (m_inner.size() != m_F.size()) m_inner.assign(m_F.size(), 0);
    std::vector<int64_t> ids(m_F.size());
    std::iota(ids.begin(), ids.end(), int64_t(0));
    if (!ids.empty()) m_root = build(ids, 0, ids.size());
}

int VisibleField::build(std::vector<int64_t>& ids, const size_t b, const size_t e)
{
    Node n;
    n.lo = Vec3::Constant(std::numeric_limits<double>::infinity());
    n.hi = Vec3::Constant(-std::numeric_limits<double>::infinity());
    Vec3 clo = n.lo, chi = n.hi;
    for (size_t i = b; i < e; ++i) {
        const auto& f = m_F[size_t(ids[i])];
        Vec3 cen = Vec3::Zero();
        for (int k = 0; k < 3; ++k) {
            const Vec3& x = m_V[size_t(f[k])];
            n.lo = n.lo.cwiseMin(x);
            n.hi = n.hi.cwiseMax(x);
            cen += x / 3.;
        }
        clo = clo.cwiseMin(cen);
        chi = chi.cwiseMax(cen);
    }
    const int id = int(m_nodes.size());
    m_nodes.push_back(n);
    if (e - b == 1) {
        m_nodes[size_t(id)].tri = ids[b];
        return id;
    }
    int axis = 0;
    (chi - clo).maxCoeff(&axis);
    const size_t m = b + (e - b) / 2;
    std::nth_element(
        ids.begin() + long(b),
        ids.begin() + long(m),
        ids.begin() + long(e),
        [&](int64_t x, int64_t y) {
            const auto& fx = m_F[size_t(x)];
            const auto& fy = m_F[size_t(y)];
            const double cx =
                m_V[size_t(fx[0])][axis] + m_V[size_t(fx[1])][axis] + m_V[size_t(fx[2])][axis];
            const double cy =
                m_V[size_t(fy[0])][axis] + m_V[size_t(fy[1])][axis] + m_V[size_t(fy[2])][axis];
            return cx < cy;
        });
    const int l = build(ids, b, m);
    const int r = build(ids, m, e);
    m_nodes[size_t(id)].left = l;
    m_nodes[size_t(id)].right = r;
    return id;
}

bool VisibleField::visible(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& q,
    const int64_t t,
    const VisibleStart& start,
    const VisibilityCells& cells) const
{
    const auto& f = m_F[size_t(t)];
    if (m_inner[size_t(t)] != 0) {
        const int s = o3(m_V[size_t(f[0])], m_V[size_t(f[1])], m_V[size_t(f[2])], p);
        if (s == 0 || s == m_inner[size_t(t)]) return false;
    }
    return walk(p, q, start_cones(start, cells), cells) == Walk::Visible;
}

VisibleField::Feature VisibleField::nearest(
    const Eigen::Vector3d& p,
    const VisibleStart& start,
    const VisibilityCells& cells) const
{
    Feature res;
    ++m_counts.queries;
    const std::vector<Cone> cones = start_cones(start, cells);
    if (m_root < 0) return res;
    if (cones.empty()) {
        ++m_counts.none_visible;
        return euclidean_nearest(p);
    }
    double best = std::numeric_limits<double>::infinity();
    // Hidden further along (assumption 2): the nearest such triangle's lower bound, as a feature.
    double unresolved = std::numeric_limits<double>::infinity();
    Feature lower_bound;
    // A walk that failed on exact rationals: reported as Stop, not assumed.
    double failed = std::numeric_limits<double>::infinity();
    int64_t failed_tri = -1;
    // Best first: boxes, and each leaf's triangle under its own squared distance, in increasing
    // order of that lower bound; the search ends when the nearest entry left is no nearer than the
    // best point proven visible. Every triangle nearer than the answer is still examined, so the
    // answer is the depth-first search's; what it saves is the triangles a nearer visible one
    // beats -- their walks and the exact path of the hidden ones. Ties pop by node index.
    struct Entry
    {
        double d2;
        int node;
        bool tri; ///< the leaf's triangle, keyed by its own distance, not its box's
    };
    const auto later = [](const Entry& x, const Entry& y) {
        return x.d2 != y.d2 ? x.d2 > y.d2 : x.node > y.node;
    };
    std::priority_queue<Entry, std::vector<Entry>, decltype(later)> queue(later);
    queue.push(
        {box_dist2(p, m_nodes[size_t(m_root)].lo, m_nodes[size_t(m_root)].hi), m_root, false});
    while (!queue.empty()) {
        const Entry en = queue.top();
        queue.pop();
        if (en.d2 >= best) break;
        const int ni = en.node;
        const Node& n = m_nodes[size_t(ni)];
        if (!en.tri) {
            std::array<Vec3, 8> corners;
            for (int k = 0; k < 8; ++k) {
                corners[size_t(k)] = Vec3(
                    (k & 1) ? n.hi[0] : n.lo[0],
                    (k & 2) ? n.hi[1] : n.lo[1],
                    (k & 4) ? n.hi[2] : n.lo[2]);
            }
            if (cone_hidden(cones, corners)) continue;
            if (n.left < 0) {
                const auto& f = m_F[size_t(n.tri)];
                int dim = -1;
                Vec3 dir;
                const Vec3 q = closest_on_triangle(
                    p,
                    m_V[size_t(f[0])],
                    m_V[size_t(f[1])],
                    m_V[size_t(f[2])],
                    dim,
                    dir);
                queue.push({(p - q).squaredNorm(), ni, true});
                continue;
            }
            for (const int ch : {n.left, n.right}) {
                queue.push(
                    {box_dist2(p, m_nodes[size_t(ch)].lo, m_nodes[size_t(ch)].hi), ch, false});
            }
            continue;
        }
        {
            const int64_t t = n.tri;
            const auto& f = m_F[size_t(t)];
            const Vec3 &a = m_V[size_t(f[0])], &b = m_V[size_t(f[1])], &c = m_V[size_t(f[2])];
            int dim = -1;
            Vec3 dir;
            const Vec3 q = closest_on_triangle(p, a, b, c, dim, dir);
            const double d2 = en.d2;
            if (m_inner[size_t(t)] != 0) {
                // A triangle bounding the input solid is reached only from its outer side; from
                // its plane or from inside, all of it is hidden.
                const int s = o3(a, b, c, p);
                if (s == 0 || s == m_inner[size_t(t)]) continue;
            }
            const std::array<Vec3, 3> tri = {{a, b, c}};
            if (cone_hidden(cones, tri)) continue;
            const Walk w = walk(p, q, cones, cells);
            if (w == Walk::Visible) {
                best = d2;
                res.status = Feature::Status::Found;
                res.foot = q;
                res.dim = dim;
                res.dir = dir;
                res.d = std::sqrt(d2);
                res.tri = t;
                continue;
            }
            // Hidden nearest point. Every visible point of the triangle lies in a start cone, so
            // its nearest point among them is the candidate: the triangle clipped by each cone,
            // exactly, then the nearest point of each piece. If that one is visible it is the
            // triangle's nearest visible point; if no cone holds any of it, all of it is hidden.
            // Exact: the nearest point of a clipped piece is rational, and it usually lies ON a
            // cone's plane (the segment grazes p's own front face), where a rounded copy would
            // fall a hair inside or outside the band at random -- so it is walked to exactly.
            const wmtk::Vector3r pr = exact_point(p, start, cells);
            double d2c = std::numeric_limits<double>::infinity();
            wmtk::Vector3r qcr;
            int qcone = -1;
            for (size_t ci = 0; ci < cones.size(); ++ci) {
                const Cone& cn = cones[ci];
                const std::vector<wmtk::Vector3r> piece = clip_by_cone(cn, a, b, c);
                if (piece.empty()) continue;
                const wmtk::Vector3r qq = nearest_on_polygon_r(pr, piece);
                const double dd = (qq - pr).dot(qq - pr).to_double();
                if (dd < d2c) {
                    d2c = dd;
                    qcr = qq;
                    qcone = int(ci);
                }
            }
            if (!std::isfinite(d2c) || d2c >= best) continue; // all hidden, or not closer
            const Vec3 qc(qcr[0].to_double(), qcr[1].to_double(), qcr[2].to_double());
            const Walk wc = walk_t<wmtk::Vector3r>(pr, qcr, cones, cells, qcone);
            if (wc == Walk::Visible) {
                best = d2c;
                res.status = Feature::Status::Found;
                res.foot = qc;
                // The feature of the triangle qc is on: its interior, an edge, or a vertex.
                int qdim = 2;
                Vec3 qdir = Vec3::Zero();
                const Vec3 bc = closest_on_triangle(qc, a, b, c, qdim, qdir);
                (void)bc;
                res.dim = qdim;
                res.dir = qdir;
                res.d = std::sqrt(d2c);
                res.tri = t;
                res.how = 1;
                continue;
            }
            if (wc == Walk::Inconsistent) {
                if (d2c < failed) {
                    failed = d2c;
                    failed_tri = t;
                }
                continue;
            }
            // Hidden further along: part of the triangle may still be visible, no closer than
            // d2c, which is the lower bound kept (assumption 2).
            if (d2c < unresolved) {
                unresolved = d2c;
                lower_bound.status = Feature::Status::Found;
                lower_bound.foot = qc;
                int qdim = 2;
                Vec3 qdir = Vec3::Zero();
                (void)closest_on_triangle(qc, a, b, c, qdim, qdir);
                lower_bound.dim = qdim;
                lower_bound.dir = qdir;
                lower_bound.d = std::sqrt(d2c);
                lower_bound.tri = t;
                lower_bound.how = 3;
            }
        }
    }
    if (failed < best) {
        res.status = Feature::Status::Stop;
        res.tri = failed_tri;
        res.d = std::sqrt(failed);
        ++m_counts.stops;
        return res;
    }
    if (unresolved < best) {
        ++m_counts.hidden_further;
        if (std::isfinite(best)) {
            m_counts.hidden_further_max_gap =
                std::max(m_counts.hidden_further_max_gap, std::sqrt(best) - lower_bound.d);
        } else {
            ++m_counts.hidden_further_unbounded;
        }
        return lower_bound;
    }
    if (res.status != Feature::Status::Found) {
        ++m_counts.none_visible;
        return euclidean_nearest(p);
    }
    if (res.how == 1) ++m_counts.first_step;
    return res;
}

VisibleField::Feature VisibleField::euclidean_nearest(const Eigen::Vector3d& p) const
{
    Feature res;
    double best = std::numeric_limits<double>::infinity();
    std::vector<int> stack = {m_root};
    while (!stack.empty()) {
        const Node& n = m_nodes[size_t(stack.back())];
        stack.pop_back();
        if (box_dist2(p, n.lo, n.hi) >= best) continue;
        if (n.left < 0) {
            const auto& f = m_F[size_t(n.tri)];
            int dim = -1;
            Vec3 dir;
            const Vec3 q = closest_on_triangle(
                p,
                m_V[size_t(f[0])],
                m_V[size_t(f[1])],
                m_V[size_t(f[2])],
                dim,
                dir);
            const double d2 = (p - q).squaredNorm();
            if (d2 < best) {
                best = d2;
                res.status = Feature::Status::Found;
                res.foot = q;
                res.dim = dim;
                res.dir = dir;
                res.d = std::sqrt(d2);
                res.tri = n.tri;
                res.how = 2;
            }
            continue;
        }
        stack.push_back(n.left);
        stack.push_back(n.right);
    }
    return res;
}

double VisibleField::relative_residual(const Eigen::Vector3d& p, const Feature& f) const
{
    return ((p - f.foot).norm() - m_delta) / m_delta;
}

Eigen::Vector3d VisibleField::gradient(const Eigen::Vector3d& p, const Feature& f) const
{
    // EuclideanOffsetPotential::gradient() with the visible feature in place of the nearest one.
    const Vec3 r = p - f.foot;
    const double d = r.norm();
    if (!(d > 1e-14)) return Vec3::Zero();
    return r / (d * m_delta);
}

Eigen::Matrix3d VisibleField::hessian(const Eigen::Vector3d& p, const Feature& f) const
{
    // EuclideanOffsetPotential::hessian() with the visible feature: 0 in a triangle's interior,
    // the cylinder around an edge, the sphere around a vertex.
    const Vec3 r = p - f.foot;
    const double d = r.norm();
    if (!(d > 1e-14)) return Eigen::Matrix3d::Zero();
    const Vec3 u = r / d;
    if (f.dim == 2) return Eigen::Matrix3d::Zero();
    if (f.dim == 1) {
        return (Eigen::Matrix3d::Identity() - f.dir * f.dir.transpose() - u * u.transpose()) /
               (d * m_delta);
    }
    return (Eigen::Matrix3d::Identity() - u * u.transpose()) / (d * m_delta);
}

} // namespace wmtk::components::topological_offset
