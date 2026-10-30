#include "SurfaceMap.hpp"

#include "scaffold2d.hpp"
#include "slim2d.hpp"

#include <wmtk/history/Replay.hpp>
#include <wmtk/utils/predicates.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <unordered_map>

namespace wmtk::components::tracking {

using history::Edit;
using history::EditKind;
using history::Replay;

namespace {

using Tri = std::array<uint64_t, 3>;
using UV = std::map<uint64_t, Eigen::Vector2d>;
using Positions = std::map<uint64_t, Eigen::Vector3d>;

constexpr uint64_t kNone = std::numeric_limits<uint64_t>::max();

/// Where a split's parent face is laid out: its corners, in stored order, on these points.
const std::array<Eigen::Vector2d, 3> kCanonical = {
    {Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0), Eigen::Vector2d(0, 1)}};

/// The barycentric coordinates of q in the triangle (a, b, c); not finite if it is degenerate.
Eigen::Vector3d barycentric(
    const Eigen::Vector2d& q,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c)
{
    const Eigen::Vector2d ab = b - a, ac = c - a, aq = q - a;
    const double det = ab.x() * ac.y() - ab.y() * ac.x();
    const double l1 = (aq.x() * ac.y() - aq.y() * ac.x()) / det;
    const double l2 = (ab.x() * aq.y() - ab.y() * aq.x()) / det;
    return {1 - l1 - l2, l1, l2};
}

/// The position the edit gave `vid`, or `fallback` if it did not move it.
Eigen::Vector3d position_after(const Edit& e, const uint64_t vid, const Eigen::Vector3d& fallback)
{
    for (const auto& m : e.moves) {
        if (m.vid == vid) return m.pos;
    }
    return fallback;
}

/// Barycentric coordinates of p, projected onto the plane of the 3D triangle (a, b, c); not
/// finite if it is degenerate. Taken against the triangle's normal: the normal equations'
/// Gram determinant would square the conditioning, losing eight digits on a sliver of aspect
/// 1e4 -- which a tet surface has after insertion -- and a point mapped there and back would
/// drift by that much.
Eigen::Vector3d barycentric3(const Eigen::Vector3d& p, const std::array<Eigen::Vector3d, 3>& t)
{
    const Eigen::Vector3d e0 = t[1] - t[0], e1 = t[2] - t[0], ep = p - t[0];
    const Eigen::Vector3d n = e0.cross(e1);
    const double nn = n.squaredNorm();
    const double l1 = ep.cross(e1).dot(n) / nn, l2 = e0.cross(ep).dot(n) / nn;
    return {1 - l1 - l2, l1, l2};
}

/// Barycentric coordinates of p in the 3D triangle (a, b, c), kept strictly inside it.
Eigen::Vector3d inside_barycentric(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
    const Eigen::Vector3d e0 = b - a, e1 = c - a, ep = p - a;
    const Eigen::Vector3d n = e0.cross(e1);
    const double nn = n.squaredNorm();
    Eigen::Vector3d l(1. / 3, 1. / 3, 1. / 3);
    if (nn > 0) {
        l[1] = ep.cross(e1).dot(n) / nn;
        l[2] = e0.cross(ep).dot(n) / nn;
        l[0] = 1 - l[1] - l[2];
    }
    for (int k = 0; k < 3; ++k) l[k] = std::max(l[k], 1e-6);
    return l / l.sum();
}

/// Tutte embedding with uniform weights: the vertices of `tris` not in `fixed` go to the average
/// of their neighbours. Returns every vertex's position.
UV tutte(const std::vector<Tri>& tris, const UV& fixed)
{
    std::map<uint64_t, std::set<uint64_t>> nbr;
    for (const Tri& t : tris) {
        for (int i = 0; i < 3; ++i) {
            nbr[t[i]].insert(t[(i + 1) % 3]);
            nbr[t[(i + 1) % 3]].insert(t[i]);
        }
    }
    std::map<uint64_t, int> unknown;
    for (const auto& [v, ns] : nbr) {
        if (!fixed.count(v)) unknown.emplace(v, int(unknown.size()));
    }
    UV uv = fixed;
    if (unknown.empty()) return uv;
    const int n = int(unknown.size());
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(n, 2);
    for (const auto& [v, i] : unknown) {
        A(i, i) = double(nbr[v].size());
        for (const uint64_t w : nbr[v]) {
            if (const auto it = unknown.find(w); it != unknown.end()) {
                A(i, it->second) -= 1;
            } else {
                b.row(i) += fixed.at(w).transpose();
            }
        }
    }
    const Eigen::MatrixXd x = A.partialPivLu().solve(b);
    for (const auto& [v, i] : unknown) uv[v] = x.row(i).transpose();
    return uv;
}

/// The boundary loop of an oriented triangle patch, in the direction of its triangles, if the
/// patch is a disk.
std::optional<std::vector<uint64_t>> disk_boundary(const std::vector<Tri>& tris)
{
    std::set<std::array<uint64_t, 2>> dir;
    std::set<uint64_t> verts;
    for (const Tri& t : tris) {
        for (int c = 0; c < 3; ++c) {
            const std::array<uint64_t, 2> e{{t[c], t[(c + 1) % 3]}};
            if (!dir.insert(e).second) return std::nullopt; // misoriented
            verts.insert(t[c]);
        }
    }
    std::map<uint64_t, uint64_t> next;
    size_t half_edges = 0;
    for (const auto& e : dir) {
        const bool twin = dir.count({{e[1], e[0]}}) > 0;
        half_edges += twin ? 1 : 2;
        if (twin) continue;
        if (!next.emplace(e[0], e[1]).second) return std::nullopt; // pinched at e[0]
    }
    if (next.size() < 3) return std::nullopt;
    std::vector<uint64_t> loop;
    uint64_t cur = next.begin()->first;
    do {
        loop.push_back(cur);
        const auto it = next.find(cur);
        if (it == next.end() || loop.size() > next.size()) return std::nullopt;
        cur = it->second;
    } while (cur != loop.front());
    if (loop.size() != next.size()) return std::nullopt; // more than one loop
    const long n_edges = long(half_edges / 2);
    if (long(verts.size()) - n_edges + long(tris.size()) != 1) return std::nullopt;
    return loop;
}

/// Coordinates of the barycentric point `b` of a face in the piece of it with corners `sub`.
Eigen::Vector3d local_coordinates(
    const std::array<Eigen::Vector3d, 3>& sub,
    const Eigen::Vector3d& b)
{
    Eigen::Matrix3d M;
    for (int k = 0; k < 3; ++k) M.col(k) = sub[k];
    return M.partialPivLu().solve(b);
}

bool positive(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c)
{
    return utils::predicates::orient2d(a, b, c) == utils::predicates::Orientation::POSITIVE;
}

/// A face, or a piece of one, in a local patch: its corners -- vertex ids, or a virtual vertex on
/// one of its edges -- and where they sit in the face, in barycentric coordinates.
struct Piece
{
    uint64_t fid;
    Tri t;
    std::array<Eigen::Vector3d, 3> sub = {
        {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()}};
};

/// The faces of a patch before or after an edit, and the 3D positions of their corners.
struct Patch
{
    std::vector<Piece> pieces;
    Positions P;

    Patch(const std::vector<uint64_t>& fs, const std::vector<Tri>& ts, Positions positions)
        : P(std::move(positions))
    {
        for (size_t i = 0; i < fs.size(); ++i) pieces.push_back({fs[i], ts[i]});
    }

    std::vector<Tri> tris() const
    {
        std::vector<Tri> ts;
        for (const Piece& p : pieces) ts.push_back(p.t);
        return ts;
    }

    /// Splits the piece holding the edge (a, b), virtually, at w = (1 - tau) a + tau b: into the
    /// piece with w for b and the one with w for a, consecutive. False if no piece holds it.
    bool split(const uint64_t a, const uint64_t b, const double tau, const uint64_t w)
    {
        for (size_t k = 0; k < pieces.size(); ++k) {
            const Tri& t = pieces[k].t;
            const size_t ia = size_t(std::find(t.begin(), t.end(), a) - t.begin());
            const size_t ib = size_t(std::find(t.begin(), t.end(), b) - t.begin());
            if (ia == 3 || ib == 3) continue;
            Piece with_a = pieces[k], with_b = pieces[k];
            const Eigen::Vector3d sub_w = (1 - tau) * pieces[k].sub[ia] + tau * pieces[k].sub[ib];
            with_a.t[ib] = w;
            with_a.sub[ib] = sub_w;
            with_b.t[ia] = w;
            with_b.sub[ia] = sub_w;
            pieces[k] = with_a;
            pieces.insert(pieces.begin() + long(k) + 1, with_b);
            P[w] = (1 - tau) * P.at(a) + tau * P.at(b);
            return true;
        }
        return false;
    }
};

/// Whether `loop` is `ids` up to where it starts.
bool same_loop(const std::vector<uint64_t>& loop, const std::vector<uint64_t>& ids)
{
    if (loop.size() != ids.size() || loop.empty()) return false;
    const auto it = std::find(loop.begin(), loop.end(), ids.front());
    if (it == loop.end()) return false;
    const size_t s = size_t(it - loop.begin());
    for (size_t i = 0; i < ids.size(); ++i) {
        if (loop[(s + i) % loop.size()] != ids[i]) return false;
    }
    return true;
}

} // namespace

SurfaceMap::SurfaceMap(const history::SurfaceHistory& h, const SurfaceMapOptions& options)
    : m_in_vertices(h.vertices)
    , m_in_faces(h.faces)
    , m_in_face_alive(h.face_alive)
{
    Replay r(h);
    // While a collapse tries the removed vertex's star alone (CollapsePatch::removed_star).
    bool removed_star_try = false;
    const auto layout = [](const Tri& t, const UV& uv) {
        return std::array<Eigen::Vector2d, 3>{{uv.at(t[0]), uv.at(t[1]), uv.at(t[2])}};
    };
    const auto corners_of = [&r](const std::vector<uint64_t>& fs) {
        std::vector<Tri> ts;
        for (const uint64_t f : fs) ts.push_back(r.faces()[f]);
        return ts;
    };
    const auto positions_of = [&r](const std::vector<Tri>& ts) {
        Positions P;
        for (const Tri& t : ts) {
            for (const uint64_t v : t) P[v] = r.vertices()[v];
        }
        return P;
    };

    // A virtual vertex: a point on an edge of a patch that stands in for a vertex the other patch
    // has there.
    constexpr uint64_t kVirtual = kNone - 1;
    using Slots = std::vector<std::pair<uint64_t, uint64_t>>;

    // Lowers the distortion of a joint embedding of `before` and `after`, keeping it one: SLIM
    // (Rabinovich et al. 2017) moves the vertices towards the 3D shapes of their triangles, and
    // a scaffold (Jiang et al. 2017) shared by both -- a ring of triangles from the outline out
    // to a fixed circle -- keeps the outline from folding over, since the line search keeps
    // every triangle, the scaffold's included, positive. The outline `slots` is free but shared:
    // each slot is one point, a corner of `before` and one of `after`. The embedding must be on
    // a circle about the origin, as the Tutte ones are; it is rescaled to the 3D area.
    const auto refine = [&](const Slots& slots,
                            const Patch& before,
                            const Patch& after,
                            UV& uv_b,
                            UV& uv_a) {
        const std::vector<Tri> before_t = before.tris(), after_t = after.tris();
        double area_3d = 0, area_2d = 0;
        for (const Tri& t : before_t) {
            const auto& P = before.P;
            area_3d += 0.5 * (P.at(t[1]) - P.at(t[0])).cross(P.at(t[2]) - P.at(t[0])).norm();
            const Eigen::Vector2d a = uv_b.at(t[1]) - uv_b.at(t[0]);
            const Eigen::Vector2d b = uv_b.at(t[2]) - uv_b.at(t[0]);
            area_2d += 0.5 * (a.x() * b.y() - a.y() * b.x());
        }
        if (!(area_3d > 0) || !(area_2d > 0)) return;
        const double scale = std::sqrt(area_3d / area_2d);

        std::vector<Eigen::Vector2d> x;
        std::vector<char> fixed;
        std::map<uint64_t, int> var_b, var_a;
        for (const auto& [b, a] : slots) {
            var_b[b] = var_a[a] = int(x.size());
            x.push_back(scale * uv_b.at(b));
            fixed.push_back(0);
        }
        const auto var = [&](std::map<uint64_t, int>& vars, const UV& uv, uint64_t v) {
            const auto [it, inserted] = vars.emplace(v, int(x.size()));
            if (inserted) {
                x.push_back(scale * uv.at(v));
                fixed.push_back(0);
            }
            return it->second;
        };
        std::vector<SlimElement> elements;
        const auto add = [&](const Patch& p, const std::vector<Tri>& ts, auto& vars, const UV& uv) {
            for (const Tri& t : ts) {
                elements.push_back(slim_element(
                    {{var(vars, uv, t[0]), var(vars, uv, t[1]), var(vars, uv, t[2])}},
                    p.P.at(t[0]),
                    p.P.at(t[1]),
                    p.P.at(t[2]),
                    1.));
            }
        };
        add(before, before_t, var_b, uv_b);
        add(after, after_t, var_a, uv_a);
        // The scaffold keeps the shared outline -- slots 0 to n - 1 -- a simple polygon, so the
        // two patches lay out the same polygon whatever SLIM does; weighted so it gives way to
        // them. See Scaffold.
        const double scaffold_weight = 1e-2 * area_3d / double(before_t.size());
        const size_t n = slots.size();
        if (options.scaffold == Scaffold::ring) {
            std::vector<int> outer(n);
            for (size_t i = 0; i < n; ++i) {
                outer[i] = int(x.size());
                x.push_back(3 * x[i]);
                fixed.push_back(1);
            }
            for (size_t i = 0; i < n; ++i) {
                const int a = int(i), b = int((i + 1) % n), c = outer[i], d = outer[(i + 1) % n];
                elements.push_back(slim_element({{a, c, d}}, x[a], x[c], x[d], scaffold_weight));
                elements.push_back(slim_element({{a, d, b}}, x[a], x[d], x[b], scaffold_weight));
            }
            slim(x, fixed, elements, 10);
        } else {
            // Rebuilt around the patch before every iteration, so SLIM's stopping rule -- an
            // iteration lowering the energy by less than 1e-4 of it -- is applied across the
            // rebuilds. box_scaffold refuses only an outline that is not simple, which the
            // scaffold itself prevents.
            const size_t n_vars = x.size(), n_patch = elements.size();
            std::vector<Eigen::Vector2d> polygon(n), corners;
            std::vector<std::array<int, 3>> tris;
            const auto var = [&](const int i) {
                return i < int(n) ? i : int(n_vars) + (i - int(n));
            };
            for (int it = 0; it < 10; ++it) {
                x.resize(n_vars);
                fixed.resize(n_vars);
                elements.resize(n_patch);
                Eigen::Vector2d lo = x[0], hi = x[0];
                for (const Eigen::Vector2d& p : x) {
                    lo = lo.cwiseMin(p);
                    hi = hi.cwiseMax(p);
                }
                const Eigen::Vector2d mid = (lo + hi) / 2, half = 1.5 * (hi - lo);
                std::copy(x.begin(), x.begin() + long(n), polygon.begin());
                if (!box_scaffold(polygon, mid - half, mid + half, corners, tris)) break;
                for (const Eigen::Vector2d& c : corners) {
                    x.push_back(c);
                    fixed.push_back(1);
                }
                for (const auto& t : tris) {
                    const std::array<int, 3> v = {{var(t[0]), var(t[1]), var(t[2])}};
                    elements.push_back(slim_element(v, x[v[0]], x[v[1]], x[v[2]], scaffold_weight));
                }
                const double before = slim_energy(x, elements);
                const double after = slim(x, fixed, elements, 1);
                if (!(before - after > 1e-4 * before)) break;
            }
        }
        for (auto& [v, uv] : uv_b) {
            if (const auto it = var_b.find(v); it != var_b.end()) uv = x[it->second];
        }
        for (auto& [v, uv] : uv_a) {
            if (const auto it = var_a.find(v); it != var_a.end()) uv = x[it->second];
        }
    };

    // The cell of a joint embedding of `before` and `after` in one planar domain, whose outline
    // both share: `slots`, in the order of before's outline, each a corner of `before` and one
    // of `after` at the same point. Tutte embeddings on the unit circle, outline spaced by 3D
    // length, start it; refine() lowers its distortion (the joint flattening of Liu et al. 2021,
    // with a shared scaffold). False if a patch's outline is not its slots.
    const auto joint = [&](const Patch& before,
                           const Patch& after,
                           const Slots& slots,
                           Cell& cell) {
        std::vector<uint64_t> outline_b, outline_a;
        for (const auto& [b, a] : slots) {
            outline_b.push_back(b);
            outline_a.push_back(a);
        }
        const std::vector<Tri> before_t = before.tris(), after_t = after.tris();
        const auto loop_b = disk_boundary(before_t), loop_a = disk_boundary(after_t);
        if (!loop_b || !loop_a || !same_loop(*loop_b, outline_b) ||
            !same_loop(*loop_a, outline_a)) {
            return false;
        }
        const size_t n = slots.size();
        std::vector<double> len(n), s(n + 1, 0.);
        for (size_t i = 0; i < n; ++i) {
            len[i] = (before.P.at(outline_b[i]) - before.P.at(outline_b[(i + 1) % n])).norm();
        }
        // A slot apart from the next by a fraction of the mean, at least, so the start is not
        // flat where the outline has a short edge; SLIM brings it back to its length.
        const double floor = 1e-2 * std::accumulate(len.begin(), len.end(), 0.) / double(n);
        for (size_t i = 0; i < n; ++i) s[i + 1] = s[i] + std::max({len[i], floor, 1e-300});
        UV fixed_b, fixed_a;
        for (size_t i = 0; i < n; ++i) {
            const double th = 2 * M_PI * s[i] / s.back();
            fixed_b[outline_b[i]] = fixed_a[outline_a[i]] =
                Eigen::Vector2d(std::cos(th), std::sin(th));
        }
        UV uv_b = tutte(before_t, fixed_b), uv_a = tutte(after_t, fixed_a);
        refine(slots, before, after, uv_b, uv_a);
        // A patch folded over in 3D -- a triangle facing against the patch -- cannot be laid flat
        // as it is: the map, still a bijection, stretches there.
        const auto folded = [](const std::vector<Tri>& ts, const Positions& Q) {
            const auto normal = [&Q](const Tri& t) {
                return Eigen::Vector3d((Q.at(t[1]) - Q.at(t[0])).cross(Q.at(t[2]) - Q.at(t[0])));
            };
            Eigen::Vector3d n = Eigen::Vector3d::Zero();
            for (const Tri& t : ts) n += normal(t);
            return std::any_of(ts.begin(), ts.end(), [&](const Tri& t) {
                return normal(t).dot(n) < 0;
            });
        };
        if (folded(before_t, before.P) || folded(after_t, after.P)) ++m_folded_cells;
        bool ok = true;
        const auto add = [&](std::vector<Face>& to, const Patch& p, const UV& uv) {
            for (const Piece& piece : p.pieces) {
                to.push_back({piece.fid, layout(piece.t, uv), piece.sub});
                ok = positive(to.back().uv[0], to.back().uv[1], to.back().uv[2]) && ok;
            }
        };
        add(cell.before, before, uv_b);
        add(cell.after, after, uv_a);
        if (!ok) ++m_build_warnings.inverted;
        return true;
    };

    // Ids of a cell's own: the pieces of a vertex a cut makes, and virtual vertices. Counted down
    // from kVirtual, far above any vertex id.
    struct LocalIds
    {
        uint64_t next = kVirtual;
        std::map<uint64_t, uint64_t> vertex; // of each, the vertex it stands for
        std::map<std::pair<uint64_t, uint64_t>, uint64_t> names; // of the pieces of vertices
        uint64_t fresh(const uint64_t v)
        {
            vertex[next] = v;
            return next--;
        }
        uint64_t vertex_of(const uint64_t id) const
        {
            const auto it = vertex.find(id);
            return it == vertex.end() ? id : it->second;
        }
    };

    // The feature edges among the edges of `ts`, now.
    const auto features_in = [&r](const std::vector<Tri>& ts) {
        std::set<std::array<uint64_t, 2>> fs;
        for (const Tri& t : ts) {
            for (int k = 0; k < 3; ++k) {
                uint64_t a = t[k], b = t[(k + 1) % 3];
                if (a > b) std::swap(a, b);
                if (r.is_feature({{a, b}})) fs.insert({{a, b}});
            }
        }
        return fs;
    };

    // Cuts the patch `ts` along the feature edges inside it -- a curve that ends inside -- so it
    // is a disk with every curve on its outline, which a joint embedding then keeps: each vertex
    // splits into its wedges, the fans of its triangles joined across edges that are not feature
    // edges, and a vertex with more than one is a corner of the outline once per wedge, each
    // with an id of its own (`P` gets their positions). A wedge is named by its vertex and the
    // vertex before it on the outline, `removed` read as `kept`, so the patches before and after
    // a collapse name a corner alike. `features` are the patch's feature edges. The patch as it
    // is if no curve ends inside it; nothing if the cut is not a disk.
    const auto cut = [](const std::vector<Tri>& ts,
                        const std::set<std::array<uint64_t, 2>>& features,
                        Positions& P,
                        LocalIds& ids,
                        const uint64_t removed,
                        const uint64_t kept) -> std::optional<std::vector<Tri>> {
        std::vector<size_t> root(3 * ts.size());
        for (size_t i = 0; i < root.size(); ++i) root[i] = i;
        const auto find = [&root](size_t i) {
            while (root[i] != i) i = root[i] = root[root[i]];
            return i;
        };
        const auto corner = [&ts](size_t f, uint64_t v) {
            return 3 * f + size_t(std::find(ts[f].begin(), ts[f].end(), v) - ts[f].begin());
        };
        std::map<std::array<uint64_t, 2>, std::vector<size_t>> edge_faces;
        for (size_t i = 0; i < ts.size(); ++i) {
            for (int k = 0; k < 3; ++k) {
                uint64_t a = ts[i][k], b = ts[i][(k + 1) % 3];
                if (a > b) std::swap(a, b);
                edge_faces[{{a, b}}].push_back(i);
            }
        }
        for (const auto& [e, fs] : edge_faces) {
            if (fs.size() != 2 || features.count(e)) continue;
            for (const uint64_t v : e) root[find(corner(fs[0], v))] = find(corner(fs[1], v));
        }
        std::map<uint64_t, std::set<size_t>> wedges;
        for (size_t c = 0; c < root.size(); ++c) wedges[ts[c / 3][c % 3]].insert(find(c));
        std::map<size_t, uint64_t> wedge_id; // temporary, for vertices with more than one
        LocalIds temp;
        for (const auto& [v, ws] : wedges) {
            if (ws.size() < 2) continue;
            for (const size_t w : ws) wedge_id[w] = temp.fresh(v);
        }
        if (wedge_id.empty()) return ts;
        std::vector<Tri> cut_ts = ts;
        for (size_t c = 0; c < root.size(); ++c) {
            if (const auto it = wedge_id.find(find(c)); it != wedge_id.end()) {
                cut_ts[c / 3][c % 3] = it->second;
            }
        }
        const auto loop = disk_boundary(cut_ts);
        if (!loop) return std::nullopt;
        std::map<uint64_t, uint64_t> final_id;
        const size_t n = loop->size();
        for (size_t i = 0; i < n; ++i) {
            const uint64_t t = (*loop)[i];
            if (!temp.vertex.count(t)) continue;
            const uint64_t v = temp.vertex_of(t);
            uint64_t before = temp.vertex_of((*loop)[(i + n - 1) % n]);
            if (before == removed) before = kept;
            const auto [it, inserted] = ids.names.emplace(std::make_pair(v, before), 0);
            if (inserted) it->second = ids.fresh(v);
            final_id[t] = it->second;
            P[it->second] = P.at(v);
        }
        for (Tri& t : cut_ts) {
            for (uint64_t& v : t) {
                if (temp.vertex.count(v)) {
                    const auto it = final_id.find(v);
                    if (it == final_id.end()) return std::nullopt; // a wedge off the outline
                    v = it->second;
                }
            }
        }
        return cut_ts;
    };

    // The cell of a collapse or swap: the patch before and after it, cut along the curves inside
    // them, on the outline of the patch before. A `removed` vertex on that outline -- on the
    // boundary or a curve, maybe more than once -- stays on it, and the triangle after the edit
    // that holds the edge replacing its two is split virtually at the same spot, so the outline
    // is the same polygon after it. `before_features` are the feature edges of the patch before
    // the edit, which the replay has already applied. False if the patch is not a disk.
    const auto flatten = [&](const std::vector<uint64_t>& before_f,
                             const std::vector<Tri>& before_t_uncut,
                             const std::set<std::array<uint64_t, 2>>& before_features,
                             const std::vector<uint64_t>& after_f,
                             const std::vector<Tri>& after_t_uncut,
                             const uint64_t removed,
                             const uint64_t kept,
                             const Positions& P_uncut,
                             const Positions& Pa_uncut,
                             Cell& cell) -> bool {
        LocalIds ids;
        Positions P = P_uncut, Pa = Pa_uncut;
        const auto before_t = cut(before_t_uncut, before_features, P, ids, removed, kept);
        const auto after_t = cut(after_t_uncut, features_in(after_t_uncut), Pa, ids, kNone, kNone);
        if (!before_t || !after_t) return false;
        const auto loop = disk_boundary(*before_t);
        if (!loop) return false;
        const Patch before(before_f, *before_t, P);
        Patch after(after_f, *after_t, Pa);
        Slots slots;
        for (const uint64_t v : *loop) slots.emplace_back(v, v);
        std::vector<size_t> on_outline;
        for (size_t i = 0; i < loop->size(); ++i) {
            if (ids.vertex_of((*loop)[i]) == removed) on_outline.push_back(i);
        }
        if (on_outline.empty()) return joint(before, after, slots, cell);

        ++m_curve_collapses;
        const auto length = [&P](uint64_t a, uint64_t b) { return (P.at(a) - P.at(b)).norm(); };
        const auto neighbours = [&loop](size_t i) {
            const size_t n = loop->size();
            return std::make_pair((*loop)[(i + n - 1) % n], (*loop)[(i + 1) % n]);
        };
        // The outline keeps the removed vertex; after the edit a virtual vertex takes its place,
        // on the edge (prev, next) at its parameter along the two edges it replaced. Where no
        // edge replaces them, the survivor came out onto the outline in its place -- a boundary
        // vertex collapsed into one inside -- and stands for it there.
        const bool kept_inside = std::none_of(loop->begin(), loop->end(), [&](uint64_t v) {
            return ids.vertex_of(v) == kept;
        });
        bool split_all = true, kept_placed = false;
        for (const size_t i : on_outline) {
            const auto [prev, next] = neighbours(i);
            const double a = length(prev, (*loop)[i]), b = length((*loop)[i], next);
            const double tau = std::clamp(a + b > 0 ? a / (a + b) : 0.5, 0.01, 0.99);
            const uint64_t w = ids.fresh(removed);
            if (after.split(prev, next, tau, w)) {
                slots[i].second = w;
            } else if (kept_inside && !kept_placed) {
                slots[i].second = kept;
                kept_placed = true;
            } else {
                split_all = false;
            }
        }
        if (split_all) return joint(before, after, slots, cell);
        if (on_outline.size() > 1 || !ids.names.empty()) return false;
        // The removed star of a curve vertex collapsed into a vertex of a sheet that is on its
        // outline: the faces after do not cover the outline before, and the chord layout below
        // would not be a bijection. Both stars map it.
        if (removed_star_try) return false;
        const size_t i_rm = on_outline.front();
        const auto [prev, next] = neighbours(i_rm);
        const double a = length(prev, removed), b = length(removed, next);
        const double tau = std::clamp(a + b > 0 ? a / (a + b) : 0.5, 0.01, 0.99);
        const std::vector<Tri>& before_tris = *before_t;
        const std::vector<Tri>& after_tris = *after_t;

        // Nothing to split: flatten the outline over a chord instead, where the triangles with
        // all three corners on it go flat.
        std::vector<uint64_t> ring = *loop;
        ring.erase(ring.begin() + long(i_rm));
        std::vector<double> s(ring.size() + 1, 0.);
        for (size_t i = 0; i < ring.size(); ++i) {
            s[i + 1] = s[i] + std::max(length(ring[i], ring[(i + 1) % ring.size()]), 1e-300);
        }
        UV fixed;
        for (size_t i = 0; i < ring.size(); ++i) {
            const double th = 2 * M_PI * s[i] / s.back();
            fixed[ring[i]] = Eigen::Vector2d(std::cos(th), std::sin(th));
        }
        UV fixed_before = fixed;
        fixed_before[removed] = (1 - tau) * fixed.at(prev) + tau * fixed.at(next);
        for (const Tri& t : before_tris) {
            int on = 0;
            for (const uint64_t v : t) on += (v == prev || v == removed || v == next);
            if (on == 3) {
                ++m_build_warnings.flat_triangle;
                break;
            }
        }
        const UV uv_before = tutte(before_tris, fixed_before);
        const UV uv_after = tutte(after_tris, fixed);
        bool ok = true;
        for (size_t i = 0; i < before_f.size(); ++i) {
            cell.before.push_back({before_f[i], layout(before_tris[i], uv_before)});
            ok = positive(
                     cell.before.back().uv[0],
                     cell.before.back().uv[1],
                     cell.before.back().uv[2]) &&
                 ok;
        }
        for (size_t i = 0; i < after_f.size(); ++i) {
            cell.after.push_back({after_f[i], layout(after_tris[i], uv_after)});
            ok = positive(
                     cell.after.back().uv[0],
                     cell.after.back().uv[1],
                     cell.after.back().uv[2]) &&
                 ok;
        }
        if (!ok) ++m_build_warnings.inverted;
        return true;
    };

    // The sides of the feature curves through a patch: its triangles joined across an edge that
    // is not a feature edge, as index groups. Each side is flattened on its own, so a curve maps
    // onto itself and a face stays on its side.
    const auto sides_of = [&r](const std::vector<Tri>& ts) {
        std::vector<size_t> root(ts.size());
        for (size_t i = 0; i < root.size(); ++i) root[i] = i;
        const auto find = [&root](size_t i) {
            while (root[i] != i) i = root[i] = root[root[i]];
            return i;
        };
        std::map<std::array<uint64_t, 2>, std::vector<size_t>> edge_faces;
        for (size_t i = 0; i < ts.size(); ++i) {
            for (int k = 0; k < 3; ++k) {
                uint64_t a = ts[i][k], b = ts[i][(k + 1) % 3];
                if (a > b) std::swap(a, b);
                edge_faces[{{a, b}}].push_back(i);
            }
        }
        for (const auto& [edge, fs] : edge_faces) {
            if (fs.size() == 2 && !r.is_feature({{edge[0], edge[1]}}))
                root[find(fs[0])] = find(fs[1]);
        }
        std::map<size_t, std::vector<size_t>> groups;
        for (size_t i = 0; i < ts.size(); ++i) groups[find(i)].push_back(i);
        std::vector<std::vector<size_t>> sides;
        for (auto& [g, members] : groups) sides.push_back(std::move(members));
        return sides;
    };

    // The cells of moving vertex v to `to` (Smoothing::flatten): one per side of its star, the
    // same faces before and after, so the points stay where they were on the surface as far as
    // the joint embedding can, instead of riding along with the vertex as they do in the paper's
    // smoothing (Smoothing::barycentric). On a curve or the boundary v keeps to the outline: both
    // its places are on it, each a virtual vertex on the edge of the patch it is not a corner of,
    // matched by arc length along the curve -- the same on both sides. False where v is on more
    // than one curve: the points then keep their barycentric coordinates, which is a bijection
    // too.
    const auto move_cells =
        [&](const uint64_t v, const Eigen::Vector3d& to, std::vector<Cell>& cells) {
            const std::vector<uint64_t> star = r.faces_of(v);
            if (star.empty()) return false;
            const std::vector<Tri> star_t = corners_of(star);
            const Positions P = positions_of(star_t);
            Positions Pa = P;
            Pa[v] = to;
            std::set<std::pair<uint64_t, uint64_t>> curve; // v's outline neighbours, per side
            for (const auto& members : sides_of(star_t)) {
                std::vector<uint64_t> fs;
                std::vector<Tri> ts;
                for (const size_t i : members) {
                    fs.push_back(star[i]);
                    ts.push_back(star_t[i]);
                }
                const auto loop = disk_boundary(ts);
                if (!loop) return false;
                Patch before(fs, ts, P), after(fs, ts, Pa);
                Slots slots;
                for (const uint64_t u : *loop) slots.emplace_back(u, u);
                const auto it = std::find(loop->begin(), loop->end(), v);
                if (it != loop->end()) {
                    const size_t i = size_t(it - loop->begin()), n = loop->size();
                    const uint64_t prev = (*loop)[(i + n - 1) % n], next = (*loop)[(i + 1) % n];
                    curve.insert(std::minmax(prev, next));
                    if (curve.size() > 1) return false;
                    const auto fraction = [&](const Positions& Q) {
                        const double a = (Q.at(prev) - Q.at(v)).norm(),
                                     b = (Q.at(v) - Q.at(next)).norm();
                        return a + b > 0 ? a / (a + b) : -1.;
                    };
                    const double f_old = fraction(P), f_new = fraction(Pa);
                    if (!(f_old > 0 && f_old < 1 && f_new > 0 && f_new < 1)) return false;
                    // Kept off the edge's ends, as a collapse's: a split there would be a piece
                    // flat in the face but not in the domain, which the map could not invert. The
                    // bounds are symmetric, so the other side of the curve, measuring from its
                    // other end, splits the edge at the same point.
                    const auto clamped = [](double tau) { return std::clamp(tau, 0.01, 0.99); };
                    if (f_new < f_old) { // v moved towards prev
                        before.split(prev, v, clamped(f_new / f_old), kVirtual);
                        after.split(v, next, clamped((f_old - f_new) / (1 - f_new)), kVirtual);
                        slots[i] = {kVirtual, v};
                        slots.insert(slots.begin() + long(i) + 1, {v, kVirtual});
                    } else if (f_new > f_old) {
                        before.split(v, next, clamped((f_new - f_old) / (1 - f_old)), kVirtual);
                        after.split(prev, v, clamped(f_old / f_new), kVirtual);
                        slots[i] = {v, kVirtual};
                        slots.insert(slots.begin() + long(i) + 1, {kVirtual, v});
                    }
                }
                Cell cell;
                if (!joint(before, after, slots, cell)) return false;
                cells.push_back(std::move(cell));
            }
            return true;
        };

    // Where flatten() cannot go: each surviving face maps onto itself by its corners, which is
    // what relabelling `removed` to the survivor does, and a face that vanished takes its points
    // with it.
    const auto relabel = [&](const std::vector<uint64_t>& before_f,
                             const std::vector<uint64_t>& after_f,
                             Step& step) {
        ++m_build_warnings.not_a_disk;
        for (const uint64_t f : before_f) {
            if (std::find(after_f.begin(), after_f.end(), f) == after_f.end()) {
                step.dead.push_back(f);
                continue;
            }
            Cell cell;
            cell.before.push_back({f, kCanonical});
            cell.after.push_back({f, kCanonical});
            step.cells.push_back(std::move(cell));
        }
    };

    for (size_t edit_index = 0; edit_index < h.edits.size(); ++edit_index) {
        const Edit& e = h.edits[edit_index];
        const size_t warnings_before = m_build_warnings.total();
        Step step{e.kind, {}, {}, {}, {}};
        switch (e.kind) {
        case EditKind::split_edge: {
            const uint64_t v0 = e.v[0], v1 = e.v[1], vn = e.v[2];
            const Eigen::Vector3d p0 = r.vertices()[v0], p1 = r.vertices()[v1];
            const Eigen::Vector3d pn = position_after(e, vn, (p0 + p1) / 2);
            const double l2 = (p1 - p0).squaredNorm();
            const double t = l2 > 0 ? std::clamp((pn - p0).dot(p1 - p0) / l2, 1e-9, 1 - 1e-9) : 0.5;
            std::vector<Tri> parents;
            for (size_t i = 0; i + 1 < e.faces.size(); i += 2) {
                parents.push_back(r.faces()[e.faces[i]]);
            }
            r.apply(e);
            for (size_t i = 0; i + 1 < e.faces.size(); i += 2) {
                const Tri& parent = parents[i / 2];
                UV uv;
                for (int k = 0; k < 3; ++k) uv[parent[k]] = kCanonical[k];
                uv[vn] = (1 - t) * uv.at(v0) + t * uv.at(v1);
                Cell cell;
                cell.before.push_back({e.faces[i], kCanonical});
                cell.after.push_back({e.faces[i], layout(r.faces()[e.faces[i]], uv)});
                cell.after.push_back({e.faces[i + 1], layout(r.faces()[e.faces[i + 1]], uv)});
                step.cells.push_back(std::move(cell));
            }
            break;
        }
        case EditKind::split_face: {
            const uint64_t f = e.faces[0], vn = e.v[3];
            const Tri parent = r.faces()[f];
            const auto& P = r.vertices();
            const Eigen::Vector3d centroid = (P[parent[0]] + P[parent[1]] + P[parent[2]]) / 3;
            const Eigen::Vector3d l = inside_barycentric(
                position_after(e, vn, centroid),
                P[parent[0]],
                P[parent[1]],
                P[parent[2]]);
            r.apply(e);
            UV uv;
            for (int k = 0; k < 3; ++k) uv[parent[k]] = kCanonical[k];
            uv[vn] = l[0] * kCanonical[0] + l[1] * kCanonical[1] + l[2] * kCanonical[2];
            Cell cell;
            cell.before.push_back({f, kCanonical});
            for (const uint64_t g : e.faces) cell.after.push_back({g, layout(r.faces()[g], uv)});
            step.cells.push_back(std::move(cell));
            break;
        }
        case EditKind::collapse_edge: {
            const uint64_t removed = e.v[0], kept = e.v[1];
            // The faces a collapse maps, read before the replay applies it.
            struct Faces
            {
                std::vector<uint64_t> f;
                std::vector<Tri> t;
                Positions P;
                std::vector<std::vector<size_t>> sides;
                std::set<std::array<uint64_t, 2>> features;
            };
            const auto faces = [&](std::vector<uint64_t> fs) {
                Faces b{std::move(fs), {}, {}, {}, {}};
                b.t = corners_of(b.f);
                b.P = positions_of(b.t);
                b.sides = sides_of(b.t);
                b.features = features_in(b.t);
                return b;
            };
            std::vector<uint64_t> union_f;
            std::set_union(
                r.faces_of(removed).begin(),
                r.faces_of(removed).end(),
                r.faces_of(kept).begin(),
                r.faces_of(kept).end(),
                std::back_inserter(union_f));
            const Faces both = faces(union_f);
            const std::optional<Faces> star = options.collapse_patch == CollapsePatch::removed_star
                                                  ? std::optional<Faces>(faces(r.faces_of(removed)))
                                                  : std::nullopt;
            size_t n_spanning = 0; // faces with both endpoints, which the collapse removes
            for (const Tri& t : both.t) {
                if (std::count(t.begin(), t.end(), removed) &&
                    std::count(t.begin(), t.end(), kept)) {
                    ++n_spanning;
                }
            }
            r.apply(e);
            const std::vector<uint64_t> after_f = r.faces_of(kept);
            if (after_f.size() + n_spanning < both.f.size()) {
                m_build_warnings.merged_faces += both.f.size() - n_spanning - after_f.size();
            }
            // One cell per side of the curves through `b`; false if a side had to be relabelled.
            const auto map_sides = [&](const Faces& b, Step& to) {
                bool all = true;
                for (const auto& members : b.sides) {
                    std::vector<uint64_t> side_before_f;
                    std::vector<Tri> side_before_t;
                    bool has_removed = false;
                    for (const size_t i : members) {
                        side_before_f.push_back(b.f[i]);
                        side_before_t.push_back(b.t[i]);
                        has_removed =
                            has_removed || std::count(b.t[i].begin(), b.t[i].end(), removed);
                    }
                    if (!has_removed) continue; // nothing on this side changes
                    std::vector<uint64_t> side_after_f;
                    for (const uint64_t f : after_f) {
                        if (std::find(side_before_f.begin(), side_before_f.end(), f) !=
                            side_before_f.end()) {
                            side_after_f.push_back(f);
                        }
                    }
                    // A side the collapse removes whole -- faces on the edge cut off by
                    // non-manifold or feature edges, faces merged into another side's -- has
                    // nothing to map onto.
                    if (side_after_f.empty()) {
                        relabel(side_before_f, side_after_f, to);
                        all = false;
                        continue;
                    }
                    Cell cell;
                    const std::vector<Tri> side_after_t = corners_of(side_after_f);
                    if (flatten(
                            side_before_f,
                            side_before_t,
                            b.features,
                            side_after_f,
                            side_after_t,
                            removed,
                            kept,
                            b.P,
                            positions_of(side_after_t),
                            cell)) {
                        to.cells.push_back(std::move(cell));
                    } else {
                        relabel(side_before_f, side_after_f, to);
                        all = false;
                    }
                }
                return all;
            };
            if (star) {
                const MapWarnings warnings = m_build_warnings;
                const size_t curve_collapses = m_curve_collapses, folded = m_folded_cells;
                Step tried{e.kind, {}, {}, {}, {}};
                removed_star_try = true;
                const bool star_ok = map_sides(*star, tried);
                removed_star_try = false;
                if (star_ok) {
                    step = std::move(tried);
                    break;
                }
                m_build_warnings = warnings;
                m_curve_collapses = curve_collapses;
                m_folded_cells = folded;
            }
            map_sides(both, step);
            break;
        }
        case EditKind::swap_edge: {
            const std::vector<uint64_t> fs = e.faces;
            const std::vector<Tri> before_t = corners_of(fs);
            const Positions P = positions_of(before_t);
            const auto before_features = features_in(before_t);
            r.apply(e);
            Cell cell;
            if (flatten(
                    fs,
                    before_t,
                    before_features,
                    fs,
                    corners_of(fs),
                    kNone,
                    kNone,
                    P,
                    P,
                    cell)) {
                step.cells.push_back(std::move(cell));
            } else {
                relabel(fs, fs, step);
            }
            break;
        }
        case EditKind::replace_faces: {
            const size_t n_added = e.v.size() / 3, n_removed = e.faces.size() - n_added;
            step.dead.assign(e.faces.begin(), e.faces.begin() + long(n_removed));
            step.born.assign(e.faces.begin() + long(n_removed), e.faces.end());
            m_build_warnings.replaced += e.faces.size();
            r.apply(e);
            break;
        }
        case EditKind::relabel: {
            // Faces keep their corners, so their points keep their coordinates: a renumbering,
            // with the faces a weld leaves degenerate gone.
            step.kind = EditKind::consolidate;
            step.old_to_new.assign(r.faces().size(), kNone);
            const auto renamed = [&e](uint64_t v) { return v < e.v.size() ? e.v[v] : kNone; };
            for (uint64_t f = 0; f < r.faces().size(); ++f) {
                if (!r.face_alive()[f]) continue;
                std::array<uint64_t, 3> t = r.faces()[f];
                for (uint64_t& v : t) v = renamed(v);
                if (t[0] == kNone || t[1] == kNone || t[2] == kNone || t[0] == t[1] ||
                    t[1] == t[2] || t[0] == t[2]) {
                    continue;
                }
                const uint64_t g = e.faces.empty()      ? step.new_to_old.size()
                                   : f < e.faces.size() ? e.faces[f]
                                                        : kNone;
                if (g == kNone) continue;
                step.old_to_new[f] = g;
                if (g >= step.new_to_old.size()) step.new_to_old.resize(g + 1, kNone);
                step.new_to_old[g] = f;
            }
            r.apply(e);
            break;
        }
        case EditKind::embed: {
            Embedding& em = step.embedding;
            const auto& P = r.vertices();
            em.before.resize(r.faces().size());
            em.to_after.assign(r.faces().size(), {});
            for (uint64_t f = 0; f < r.faces().size(); ++f) {
                if (!r.face_alive()[f]) continue;
                for (int k = 0; k < 3; ++k) em.before[f][k] = P[r.faces()[f][k]];
            }
            std::vector<Eigen::Vector3d> Q;
            for (const auto& m : e.moves) {
                if (m.vid >= Q.size()) Q.resize(m.vid + 1, Eigen::Vector3d::Zero());
                Q[m.vid] = m.pos;
            }
            const size_t nf = e.v.size() / 3;
            em.after.resize(nf);
            em.to_before.resize(nf);
            for (size_t f = 0, i = 0; f < nf; ++f) {
                for (int k = 0; k < 3; ++k) em.after[f][k] = Q[e.v[3 * f + k]];
                const uint64_t n = e.faces[i++];
                for (uint64_t j = 0; j < n; ++j) {
                    const uint64_t g = e.faces[i++];
                    if (g >= em.to_after.size() || !r.face_alive()[g]) continue;
                    em.to_before[f].push_back(g);
                    em.to_after[g].push_back(f);
                }
            }
            step.kind = EditKind::embed;
            r.apply(e);
            break;
        }
        case EditKind::consolidate: {
            step.old_to_new.assign(r.faces().size(), kNone);
            for (uint64_t f = 0; f < r.faces().size(); ++f) {
                if (!r.face_alive()[f]) continue;
                step.old_to_new[f] = step.new_to_old.size();
                step.new_to_old.push_back(f);
            }
            r.apply(e);
            break;
        }
        case EditKind::move_vertices: {
            // One vertex at a time, each its own step.
            for (const history::VertexMove& m : e.moves) {
                std::vector<Cell> cells;
                const bool mapped = m.vid < r.vertex_alive().size() && r.vertex_alive()[m.vid] &&
                                    r.vertices()[m.vid] != m.pos &&
                                    options.smoothing == Smoothing::flatten &&
                                    move_cells(m.vid, m.pos, cells);
                r.apply(Edit{EditKind::move_vertices, {}, {}, {m}});
                if (!mapped) {
                    ++m_kept_moves;
                    continue;
                }
                ++m_cells_by_kind[size_t(EditKind::move_vertices)];
                m_steps.push_back({EditKind::move_vertices, std::move(cells), {}, {}, {}});
                m_steps.back().edit = edit_index;
            }
            break;
        }
        default: r.apply(e); break;
        }
        if (m_build_warnings.total() > warnings_before) m_warned_edits.push_back(edit_index);
        if (step.cells.empty() && step.dead.empty() && step.born.empty() &&
            step.kind != EditKind::consolidate && step.kind != EditKind::embed) {
            continue;
        }
        ++m_cells_by_kind[size_t(e.kind) % m_cells_by_kind.size()];
        m_steps.push_back(std::move(step));
        m_steps.back().edit = edit_index;
    }

    m_out_vertices = r.vertices();
    m_out_faces = r.faces();
    m_out_face_alive = r.face_alive();
    m_out_features = r.features();
}

std::vector<SurfacePoint> SurfaceMap::to_input(
    std::vector<SurfacePoint> points,
    MapWarnings* warnings) const
{
    return map(std::move(points), false, warnings);
}

std::vector<SurfacePoint> SurfaceMap::to_output(
    std::vector<SurfacePoint> points,
    MapWarnings* warnings) const
{
    return map(std::move(points), true, warnings);
}

std::vector<SurfacePoint>
SurfaceMap::map(std::vector<SurfacePoint> pts, const bool forward, MapWarnings* warnings) const
{
    // Counted for this call alone: the map itself is never written, so several threads may map
    // through it at once.
    MapWarnings w;
    std::unordered_map<uint64_t, std::vector<size_t>> on_face;
    // A point must start on a live face of its side: any other id would be carried along by
    // whichever face a later edit gives that slot.
    const auto& alive = forward ? m_in_face_alive : m_out_face_alive;
    for (size_t i = 0; i < pts.size(); ++i) {
        if (pts[i].face < alive.size() && alive[pts[i].face]) {
            on_face[pts[i].face].push_back(i);
        } else {
            pts[i].face = kNone;
            ++w.unlocated;
        }
    }
    std::vector<std::pair<uint64_t, size_t>> landed;

    const auto visit = [&](const Step& s) {
        if (s.kind == EditKind::consolidate) {
            const auto& ids = forward ? s.old_to_new : s.new_to_old;
            std::unordered_map<uint64_t, std::vector<size_t>> renamed;
            for (auto& [f, on] : on_face) {
                const uint64_t g = f < ids.size() ? ids[f] : kNone;
                if (g == kNone) w.unlocated += on.size();
                for (const size_t i : on) pts[i].face = g;
                if (g != kNone) renamed[g] = std::move(on);
            }
            on_face = std::move(renamed);
            return;
        }
        if (s.kind == EditKind::embed) {
            // Each point stays where it is in space, on whichever face of the other side that
            // lies on its own holds it best.
            const Embedding& em = s.embedding;
            const auto& from = forward ? em.before : em.after;
            const auto& to = forward ? em.after : em.before;
            const auto& candidates = forward ? em.to_after : em.to_before;
            std::unordered_map<uint64_t, std::vector<size_t>> moved;
            for (auto& [f, on] : on_face) {
                for (const size_t i : on) {
                    if (f >= candidates.size() || candidates[f].empty()) {
                        ++w.unlocated;
                        pts[i].face = kNone;
                        continue;
                    }
                    const auto& c = from[f];
                    const Eigen::Vector3d p =
                        pts[i].bary[0] * c[0] + pts[i].bary[1] * c[1] + pts[i].bary[2] * c[2];
                    double best = -std::numeric_limits<double>::infinity();
                    uint64_t best_g = candidates[f].front();
                    Eigen::Vector3d best_l(1. / 3, 1. / 3, 1. / 3);
                    for (const uint64_t g : candidates[f]) {
                        const Eigen::Vector3d l = barycentric3(p, to[g]);
                        if (std::isfinite(l.minCoeff()) && l.minCoeff() > best) {
                            best = l.minCoeff();
                            best_g = g;
                            best_l = l;
                        }
                    }
                    if (!(best >= -1e-9)) ++w.unlocated;
                    best_l = best_l.cwiseMax(0.);
                    best_l = best_l.sum() > 0 ? Eigen::Vector3d(best_l / best_l.sum())
                                              : Eigen::Vector3d(1. / 3, 1. / 3, 1. / 3);
                    pts[i] = {best_g, best_l};
                    moved[best_g].push_back(i);
                }
            }
            on_face = std::move(moved);
            return;
        }
        landed.clear();
        for (const Cell& c : s.cells) {
            const auto& from = forward ? c.before : c.after;
            const auto& to = forward ? c.after : c.before;
            // The pieces of one face are consecutive.
            for (size_t first = 0; first < from.size();) {
                size_t last = first + 1;
                while (last < from.size() && from[last].fid == from[first].fid) ++last;
                const auto it = on_face.find(from[first].fid);
                if (it != on_face.end()) {
                    for (const size_t i : it->second) {
                        // The piece of the face holding the point, and the point in the domain.
                        size_t piece = first;
                        Eigen::Vector3d l = pts[i].bary;
                        if (last - first > 1) {
                            double best = -std::numeric_limits<double>::infinity();
                            for (size_t k = first; k < last; ++k) {
                                const Eigen::Vector3d lk =
                                    local_coordinates(from[k].sub, pts[i].bary);
                                if (lk.minCoeff() > best) {
                                    best = lk.minCoeff();
                                    piece = k;
                                    l = lk;
                                }
                            }
                        }
                        const auto& f = from[piece];
                        const Eigen::Vector2d q = l[0] * f.uv[0] + l[1] * f.uv[1] + l[2] * f.uv[2];
                        double best = -std::numeric_limits<double>::infinity();
                        size_t best_k = 0;
                        Eigen::Vector3d best_l(1. / 3, 1. / 3, 1. / 3);
                        for (size_t k = 0; k < to.size(); ++k) {
                            const Eigen::Vector3d lk =
                                barycentric(q, to[k].uv[0], to[k].uv[1], to[k].uv[2]);
                            const double score = lk.minCoeff();
                            if (std::isfinite(score) && score > best) {
                                best = score;
                                best_k = k;
                                best_l = lk;
                            }
                        }
                        if (!(best >= -1e-9)) {
                            ++w.unlocated;
                            best_l = best_l.cwiseMax(0.);
                            if (best_l.sum() > 0) {
                                best_l /= best_l.sum();
                            } else {
                                best_l = Eigen::Vector3d(1. / 3, 1. / 3, 1. / 3);
                            }
                        }
                        const auto& g = to[best_k];
                        pts[i] = {
                            g.fid,
                            best_l[0] * g.sub[0] + best_l[1] * g.sub[1] + best_l[2] * g.sub[2]};
                        landed.emplace_back(g.fid, i);
                    }
                    on_face.erase(it);
                }
                first = last;
            }
        }
        for (const uint64_t f : forward ? s.dead : s.born) {
            const auto it = on_face.find(f);
            if (it == on_face.end()) continue;
            w.unlocated += it->second.size();
            for (const size_t i : it->second) pts[i].face = kNone;
            on_face.erase(it);
        }
        for (const auto& [f, i] : landed) on_face[f].push_back(i);
    };

    if (forward) {
        for (const Step& s : m_steps) visit(s);
    } else {
        for (auto it = m_steps.rbegin(); it != m_steps.rend(); ++it) visit(*it);
    }
    if (warnings) *warnings = w;
    return pts;
}

std::vector<const SurfaceMap::Cell*> SurfaceMap::cells_of(const size_t edit) const
{
    std::vector<const Cell*> out;
    auto it = std::lower_bound(m_steps.begin(), m_steps.end(), edit, [](const Step& s, size_t e) {
        return s.edit < e;
    });
    for (; it != m_steps.end() && it->edit == edit; ++it) {
        for (const Cell& c : it->cells) out.push_back(&c);
    }
    return out;
}

} // namespace wmtk::components::tracking
