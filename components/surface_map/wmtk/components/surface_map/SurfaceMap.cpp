#include "SurfaceMap.hpp"

#include <wmtk/history/Replay.hpp>
#include <wmtk/utils/predicates.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <unordered_map>

namespace wmtk::components::surface_map {

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

/// Barycentric coordinates of p in the 3D triangle (a, b, c), kept strictly inside it.
Eigen::Vector3d inside_barycentric(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
    const Eigen::Vector3d e0 = b - a, e1 = c - a, ep = p - a;
    const double d00 = e0.dot(e0), d01 = e0.dot(e1), d11 = e1.dot(e1);
    const double d20 = ep.dot(e0), d21 = ep.dot(e1);
    const double den = d00 * d11 - d01 * d01;
    Eigen::Vector3d l(1. / 3, 1. / 3, 1. / 3);
    if (den > 0) {
        l[1] = (d11 * d20 - d01 * d21) / den;
        l[2] = (d00 * d21 - d01 * d20) / den;
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

} // namespace

SurfaceMap::SurfaceMap(const history::SurfaceHistory& h)
    : m_in_vertices(h.vertices)
    , m_in_faces(h.faces)
    , m_in_face_alive(h.face_alive)
{
    Replay r(h);
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

    // A joint Tutte embedding of the patch before and after an edit, on the outline of the patch
    // before it, spaced on the unit circle by 3D length. A `removed` vertex on that outline -- on
    // the boundary or a curve -- stays on the circle, and the triangle after the edit that holds
    // the edge replacing its two is split virtually at the same spot, so the outline is the same
    // polygon after it. False if the patch is not a disk.
    const auto flatten = [&](const std::vector<uint64_t>& before_f,
                             const std::vector<Tri>& before_t,
                             const std::vector<uint64_t>& after_f,
                             const std::vector<Tri>& after_t,
                             const uint64_t removed,
                             const Positions& P,
                             Cell& cell) -> bool {
        const auto loop = disk_boundary(before_t);
        if (!loop) return false;
        const auto length = [&P](uint64_t a, uint64_t b) { return (P.at(a) - P.at(b)).norm(); };
        const auto on_circle = [&](const std::vector<uint64_t>& ring) {
            std::vector<double> s(ring.size() + 1, 0.);
            for (size_t i = 0; i < ring.size(); ++i) {
                s[i + 1] = s[i] + std::max(length(ring[i], ring[(i + 1) % ring.size()]), 1e-300);
            }
            UV uv;
            for (size_t i = 0; i < ring.size(); ++i) {
                const double th = 2 * M_PI * s[i] / s.back();
                uv[ring[i]] = Eigen::Vector2d(std::cos(th), std::sin(th));
            }
            return uv;
        };
        const auto add = [&](std::vector<Face>& to, uint64_t fid, const Tri& t, const UV& uv) {
            to.push_back({fid, layout(t, uv)});
            return positive(to.back().uv[0], to.back().uv[1], to.back().uv[2]);
        };

        const auto it = std::find(loop->begin(), loop->end(), removed);
        bool ok = true;
        if (it == loop->end()) {
            const UV fixed = on_circle(*loop);
            const UV uv_before = tutte(before_t, fixed);
            const UV uv_after = tutte(after_t, fixed);
            for (size_t i = 0; i < before_f.size(); ++i) {
                ok = add(cell.before, before_f[i], before_t[i], uv_before) && ok;
            }
            for (size_t i = 0; i < after_f.size(); ++i) {
                ok = add(cell.after, after_f[i], after_t[i], uv_after) && ok;
            }
            if (!ok) ++m_build_warnings.inverted;
            return true;
        }

        ++m_curve_collapses;
        const size_t i_rm = size_t(it - loop->begin());
        const uint64_t prev = (*loop)[(i_rm + loop->size() - 1) % loop->size()];
        const uint64_t next = (*loop)[(i_rm + 1) % loop->size()];
        const double a = length(prev, removed), b = length(removed, next);
        const double tau = std::clamp(a + b > 0 ? a / (a + b) : 0.5, 0.01, 0.99);
        // The triangle after the edit holding the edge (prev, next), which replaces the removed
        // vertex's two edges along the outline.
        size_t j = after_t.size();
        for (size_t k = 0; k < after_t.size(); ++k) {
            if (std::count(after_t[k].begin(), after_t[k].end(), prev) &&
                std::count(after_t[k].begin(), after_t[k].end(), next)) {
                j = k;
            }
        }
        if (j == after_t.size()) {
            // Nothing to split: flatten the outline over a chord instead, where the triangles
            // with all three corners on it go flat.
            std::vector<uint64_t> ring = *loop;
            ring.erase(ring.begin() + long(i_rm));
            const UV fixed = on_circle(ring);
            UV fixed_before = fixed;
            fixed_before[removed] = (1 - tau) * fixed.at(prev) + tau * fixed.at(next);
            for (const Tri& t : before_t) {
                int on = 0;
                for (const uint64_t v : t) on += (v == prev || v == removed || v == next);
                if (on == 3) {
                    ++m_build_warnings.flat_triangle;
                    break;
                }
            }
            const UV uv_before = tutte(before_t, fixed_before);
            const UV uv_after = tutte(after_t, fixed);
            for (size_t i = 0; i < before_f.size(); ++i) {
                ok = add(cell.before, before_f[i], before_t[i], uv_before) && ok;
            }
            for (size_t i = 0; i < after_f.size(); ++i) {
                ok = add(cell.after, after_f[i], after_t[i], uv_after) && ok;
            }
            if (!ok) ++m_build_warnings.inverted;
            return true;
        }

        // The outline keeps the removed vertex; after the edit a virtual vertex takes its place,
        // on the edge (prev, next) at its parameter along the two edges it replaced.
        constexpr uint64_t kVirtual = kNone - 1;
        const UV fixed_before = on_circle(*loop);
        UV fixed_after = fixed_before;
        fixed_after.erase(removed);
        fixed_after[kVirtual] = fixed_before.at(removed);
        const Tri split = after_t[j];
        Tri half_prev = split, half_next = split; // (prev, q, z) and (q, next, z), in place
        for (uint64_t& v : half_prev) {
            if (v == next) v = kVirtual;
        }
        for (uint64_t& v : half_next) {
            if (v == prev) v = kVirtual;
        }
        std::vector<Tri> virtual_after = after_t;
        virtual_after[j] = half_prev;
        virtual_after.push_back(half_next);
        const UV uv_before = tutte(before_t, fixed_before);
        const UV uv_after = tutte(virtual_after, fixed_after);
        for (size_t i = 0; i < before_f.size(); ++i) {
            ok = add(cell.before, before_f[i], before_t[i], uv_before) && ok;
        }
        for (size_t i = 0; i < after_f.size(); ++i) {
            if (i != j) {
                ok = add(cell.after, after_f[i], after_t[i], uv_after) && ok;
                continue;
            }
            for (const Tri& half : {half_prev, half_next}) {
                ok = add(cell.after, after_f[i], half, uv_after) && ok;
                for (int k = 0; k < 3; ++k) {
                    Eigen::Vector3d sub = Eigen::Vector3d::Zero();
                    if (half[k] != kVirtual) {
                        sub[k] = 1;
                    } else {
                        for (int m = 0; m < 3; ++m) {
                            if (split[m] == prev) sub[m] = 1 - tau;
                            if (split[m] == next) sub[m] = tau;
                        }
                    }
                    cell.after.back().sub[k] = sub;
                }
            }
        }
        if (!ok) ++m_build_warnings.inverted;
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
            std::vector<uint64_t> before_f;
            std::set_union(
                r.faces_of(removed).begin(),
                r.faces_of(removed).end(),
                r.faces_of(kept).begin(),
                r.faces_of(kept).end(),
                std::back_inserter(before_f));
            const std::vector<Tri> before_t = corners_of(before_f);
            const Positions P = positions_of(before_t);
            size_t n_spanning = 0; // faces with both endpoints, which the collapse removes
            for (const Tri& t : before_t) {
                if (std::count(t.begin(), t.end(), removed) &&
                    std::count(t.begin(), t.end(), kept)) {
                    ++n_spanning;
                }
            }
            // The sides of the feature curves through the patch: faces joined across an edge
            // that is not a feature edge. Each side is flattened on its own, so a curve maps
            // onto itself and a face stays on its side.
            std::vector<size_t> root(before_f.size());
            for (size_t i = 0; i < root.size(); ++i) root[i] = i;
            const auto find = [&root](size_t i) {
                while (root[i] != i) i = root[i] = root[root[i]];
                return i;
            };
            {
                std::map<std::array<uint64_t, 2>, std::vector<size_t>> edge_faces;
                for (size_t i = 0; i < before_t.size(); ++i) {
                    for (int k = 0; k < 3; ++k) {
                        uint64_t a = before_t[i][k], b = before_t[i][(k + 1) % 3];
                        if (a > b) std::swap(a, b);
                        edge_faces[{{a, b}}].push_back(i);
                    }
                }
                for (const auto& [edge, fs] : edge_faces) {
                    if (fs.size() == 2 && !r.is_feature(edge[0], edge[1])) {
                        root[find(fs[0])] = find(fs[1]);
                    }
                }
            }
            r.apply(e);
            const std::vector<uint64_t> after_f = r.faces_of(kept);
            if (after_f.size() + n_spanning < before_f.size()) {
                m_build_warnings.merged_faces += before_f.size() - n_spanning - after_f.size();
            }
            std::map<size_t, std::vector<size_t>> sides;
            for (size_t i = 0; i < before_f.size(); ++i) sides[find(i)].push_back(i);
            for (const auto& [side_root, members] : sides) {
                std::vector<uint64_t> side_before_f;
                std::vector<Tri> side_before_t;
                bool has_removed = false;
                for (const size_t i : members) {
                    side_before_f.push_back(before_f[i]);
                    side_before_t.push_back(before_t[i]);
                    has_removed =
                        has_removed || std::count(before_t[i].begin(), before_t[i].end(), removed);
                }
                if (!has_removed) continue; // nothing on this side changes
                std::vector<uint64_t> side_after_f;
                for (const uint64_t f : after_f) {
                    if (std::find(side_before_f.begin(), side_before_f.end(), f) !=
                        side_before_f.end()) {
                        side_after_f.push_back(f);
                    }
                }
                Cell cell;
                if (flatten(
                        side_before_f,
                        side_before_t,
                        side_after_f,
                        corners_of(side_after_f),
                        removed,
                        P,
                        cell)) {
                    step.cells.push_back(std::move(cell));
                } else {
                    relabel(side_before_f, side_after_f, step);
                }
            }
            break;
        }
        case EditKind::swap_edge: {
            const std::vector<uint64_t> fs = e.faces;
            const std::vector<Tri> before_t = corners_of(fs);
            const Positions P = positions_of(before_t);
            r.apply(e);
            Cell cell;
            if (flatten(fs, before_t, fs, corners_of(fs), kNone, P, cell)) {
                step.cells.push_back(std::move(cell));
            } else {
                relabel(fs, fs, step);
            }
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
        case EditKind::move_vertices: // maps every point onto itself
        default: r.apply(e); break;
        }
        if (m_build_warnings.total() > warnings_before) m_warned_edits.push_back(edit_index);
        if (step.cells.empty() && step.dead.empty() && e.kind != EditKind::consolidate) continue;
        ++m_cells_by_kind[size_t(e.kind) % m_cells_by_kind.size()];
        m_steps.push_back(std::move(step));
    }

    m_out_vertices = r.vertices();
    m_out_faces = r.faces();
    m_out_face_alive = r.face_alive();
    m_out_features = r.feature_edges();
}

std::vector<SurfacePoint> SurfaceMap::to_input(std::vector<SurfacePoint> points) const
{
    return map(std::move(points), false);
}

std::vector<SurfacePoint> SurfaceMap::to_output(std::vector<SurfacePoint> points) const
{
    return map(std::move(points), true);
}

std::vector<SurfacePoint> SurfaceMap::map(std::vector<SurfacePoint> pts, const bool forward) const
{
    m_map_warnings = {};
    std::unordered_map<uint64_t, std::vector<size_t>> on_face;
    for (size_t i = 0; i < pts.size(); ++i) on_face[pts[i].face].push_back(i);
    std::vector<std::pair<uint64_t, size_t>> landed;

    const auto visit = [&](const Step& s) {
        if (s.kind == EditKind::consolidate) {
            const auto& ids = forward ? s.old_to_new : s.new_to_old;
            std::unordered_map<uint64_t, std::vector<size_t>> renamed;
            for (auto& [f, on] : on_face) {
                const uint64_t g = f < ids.size() ? ids[f] : kNone;
                if (g == kNone) m_map_warnings.unlocated += on.size();
                for (const size_t i : on) pts[i].face = g;
                if (g != kNone) renamed[g] = std::move(on);
            }
            on_face = std::move(renamed);
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
                            ++m_map_warnings.unlocated;
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
        if (forward) {
            for (const uint64_t f : s.dead) {
                const auto it = on_face.find(f);
                if (it == on_face.end()) continue;
                m_map_warnings.unlocated += it->second.size();
                for (const size_t i : it->second) pts[i].face = kNone;
                on_face.erase(it);
            }
        }
        for (const auto& [f, i] : landed) on_face[f].push_back(i);
    };

    if (forward) {
        for (const Step& s : m_steps) visit(s);
    } else {
        for (auto it = m_steps.rbegin(); it != m_steps.rend(); ++it) visit(*it);
    }
    return pts;
}

} // namespace wmtk::components::surface_map
