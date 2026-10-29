#include "CurveMap.hpp"

#include <wmtk/history/Replay.hpp>

#include <algorithm>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>

namespace wmtk::components::tracking {

namespace {

using history::CurveReplay;
using history::Edit;
using history::EditKind;
using Segment = std::array<uint64_t, 2>;
constexpr uint64_t kNone = history::kNoVertex;

/// A segment's corners and their positions.
struct Geometry
{
    Segment corners;
    std::array<Eigen::Vector3d, 2> p;
    bool operator==(const Geometry& o) const { return corners == o.corners && p == o.p; }
};

Geometry geometry(const CurveReplay& r, const uint64_t f)
{
    const Segment& c = r.faces()[f];
    return {c, {{r.vertices()[c[0]], r.vertices()[c[1]]}}};
}

/// What a vertex is on one side of an operation.
struct VertexState
{
    bool alive = false;
    size_t valence = 0;
    bool feature = false;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
};

VertexState state_of(const CurveReplay& r, const uint64_t v)
{
    VertexState s;
    if (v >= r.vertices().size() || !r.vertex_alive()[v]) return s;
    s.alive = true;
    s.valence = r.faces_of(v).size();
    s.feature = r.is_feature({{v}});
    s.p = r.vertices()[v];
    return s;
}

/// The parameter of the point of segment (a, b) nearest x, in [0, 1].
double parameter(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& x)
{
    const Eigen::Vector3d d = b - a;
    const double l2 = d.squaredNorm();
    if (!(l2 > 0)) return 0;
    return std::clamp((x - a).dot(d) / l2, 0., 1.);
}

Eigen::Vector2d bary_of(const double t)
{
    return Eigen::Vector2d(1 - t, t);
}

template <typename T>
const T* find_in(const std::vector<std::pair<uint64_t, T>>& sorted, const uint64_t key)
{
    const auto it = std::lower_bound(
        sorted.begin(),
        sorted.end(),
        key,
        [](const std::pair<uint64_t, T>& e, uint64_t k) { return e.first < k; });
    return it != sorted.end() && it->first == key ? &it->second : nullptr;
}

} // namespace

CurveMap::CurveMap(const history::CurveHistory& h, const CurveMapOptions& options)
{
    CurveReplay r(h);
    m_in_vertices = r.vertices();
    m_in_segments = r.faces();
    m_in_alive = r.face_alive();
    for (const auto& f : h.features) m_features.push_back({{f[0], f[0]}});
    const bool flatten = options.smoothing == Smoothing::flatten;

    for (size_t i = 0; i < h.edits.size(); ++i) {
        const Edit& e = h.edits[i];
        const size_t warned = m_build_warnings.total();
        // Feature points follow their vertices: what a consolidation ranks them to, and where an
        // embed finds them again.
        std::vector<uint64_t> rank;
        std::vector<Eigen::Vector3d> at;
        if (e.kind == EditKind::consolidate) {
            rank.assign(r.vertices().size(), kNone);
            uint64_t n = 0;
            for (uint64_t v = 0; v < rank.size(); ++v) {
                if (r.vertex_alive()[v]) rank[v] = n++;
            }
        }
        if (e.kind == EditKind::embed) {
            for (const auto& [in, v] : m_features) {
                at.push_back(v == kNone ? Eigen::Vector3d::Zero() : r.vertices()[v]);
            }
        }

        Step step;
        step.kind = e.kind;
        switch (e.kind) {
        case EditKind::consolidate:
        case EditKind::relabel: build_renumbering(r, e, step); break;
        case EditKind::embed: build_embed(r, e, step); break;
        case EditKind::replace_faces: build_replace(r, e, step); break;
        default: build_local(r, e, step, flatten); break;
        }

        std::map<std::array<double, 3>, uint64_t> vertex_at;
        if (e.kind == EditKind::embed) {
            for (uint64_t v = 0; v < r.vertices().size(); ++v) {
                const Eigen::Vector3d& p = r.vertices()[v];
                if (r.vertex_alive()[v])
                    vertex_at.emplace(std::array<double, 3>{{p[0], p[1], p[2]}}, v);
            }
        }
        for (size_t k = 0; k < m_features.size(); ++k) {
            uint64_t& v = m_features[k][1];
            if (v == kNone) continue;
            switch (e.kind) {
            case EditKind::collapse_edge:
                if (v == e.v[0]) v = e.v[1];
                break;
            case EditKind::relabel: v = v < e.v.size() ? e.v[v] : kNone; break;
            case EditKind::consolidate: v = v < rank.size() ? rank[v] : kNone; break;
            case EditKind::embed: {
                const auto it = vertex_at.find({{at[k][0], at[k][1], at[k][2]}});
                v = it == vertex_at.end() ? kNone : it->second;
                break;
            }
            default: break;
            }
            if (v != kNone && (v >= r.vertex_alive().size() || !r.vertex_alive()[v])) v = kNone;
        }

        if (m_build_warnings.total() > warned) m_warned_edits.push_back(i);
        const bool identity = step.chains.empty() && step.forward.empty() &&
                              step.backward.empty() && step.old_to_new.empty() &&
                              step.to_after.empty();
        if (!identity) m_steps.push_back(std::move(step));
    }

    m_out_vertices = r.vertices();
    m_out_segments = r.faces();
    m_out_alive = r.face_alive();
    for (const auto& f : r.features()) m_out_features.push_back(f[0]);
}

void CurveMap::build_local(CurveReplay& r, const Edit& e, Step& step, const bool flatten)
{
    // The vertices whose segments may change, and what the operation renames.
    std::vector<uint64_t> touched;
    uint64_t removed = kNone, kept = kNone;
    if (e.kind == EditKind::split_edge) touched = {e.v[0], e.v[1], e.v[2]};
    if (e.kind == EditKind::collapse_edge) {
        removed = e.v[0];
        kept = e.v[1];
        touched = {removed, kept};
    }
    for (const auto& m : e.moves) touched.push_back(m.vid);
    std::sort(touched.begin(), touched.end());
    touched.erase(std::unique(touched.begin(), touched.end()), touched.end());
    const auto renamed = [&](const uint64_t v) { return v == removed ? kept : v; };

    // The segments at them, before and after, and the state of their vertices.
    std::map<uint64_t, Geometry> before, after;
    std::map<uint64_t, VertexState> vb, va;
    const auto gather = [&](std::map<uint64_t, Geometry>& segs) {
        for (const uint64_t t : touched) {
            if (t >= r.vertices().size() || !r.vertex_alive()[t]) continue;
            for (const uint64_t f : r.faces_of(t)) segs.emplace(f, geometry(r, f));
        }
    };
    gather(before);
    std::set<uint64_t> vertices(touched.begin(), touched.end());
    for (const auto& [f, g] : before) vertices.insert(g.corners.begin(), g.corners.end());
    for (const uint64_t v : vertices) vb[v] = state_of(r, v);
    r.apply(e);
    gather(after);
    for (const auto& [f, g] : after) vertices.insert(g.corners.begin(), g.corners.end());
    for (const uint64_t v : vertices) va[v] = state_of(r, v);
    const auto state = [](const std::map<uint64_t, VertexState>& m, const uint64_t v) {
        const auto it = m.find(v);
        return it == m.end() ? VertexState() : it->second;
    };

    // Only segments whose corners or positions change take part -- and, where a collapse moves a
    // junction, an end or a feature point onto the survivor, every segment at the survivor: the
    // chain the collapsed segment was on goes on through it.
    const auto ends_chains = [](const VertexState& s) { return s.feature || s.valence != 2; };
    const bool moves_anchor = e.kind == EditKind::collapse_edge && ends_chains(state(vb, removed));
    std::set<uint64_t> changed_before, changed_after;
    for (const auto& [f, g] : before) {
        const auto it = after.find(f);
        const bool at_kept = moves_anchor && (g.corners[0] == kept || g.corners[1] == kept);
        if (at_kept || it == after.end() || !(it->second == g)) changed_before.insert(f);
    }
    for (const auto& [f, g] : after) {
        const auto it = before.find(f);
        const bool at_kept = moves_anchor && (g.corners[0] == kept || g.corners[1] == kept);
        if (at_kept || it == before.end() || !(it->second == g)) changed_after.insert(f);
    }
    if (changed_before.empty() && changed_after.empty()) return;
    // Barycentric smoothing: every point keeps its segment and coordinates.
    if (e.kind == EditKind::move_vertices && !flatten) return;
    ++m_steps_by_kind[size_t(e.kind)];
    // A collapse along no segment welds two places on the curves into one: the chains may still
    // pair up, but two points now map onto one.
    bool welded = false;
    if (e.kind == EditKind::collapse_edge) {
        bool on_curves = false, spanned = false;
        for (const auto& [f, g] : before) {
            const bool at_removed = g.corners[0] == removed || g.corners[1] == removed;
            const bool at_kept = g.corners[0] == kept || g.corners[1] == kept;
            on_curves = on_curves || at_removed;
            spanned = spanned || (at_removed && at_kept);
        }
        welded = on_curves && !spanned;
    }

    std::map<uint64_t, size_t> changed_at_before, changed_at_after;
    for (const uint64_t f : changed_before) {
        for (const uint64_t v : before.at(f).corners) ++changed_at_before[v];
    }
    for (const uint64_t f : changed_after) {
        for (const uint64_t v : after.at(f).corners) ++changed_at_after[v];
    }
    // A chain ends where the curve does not simply pass through a vertex that moves or goes:
    // at a junction, an end or a feature point, at a vertex with a segment the operation leaves
    // as it is, and at one that stays where it was -- except, before a collapse that moves a
    // junction, an end or a feature point onto it, the survivor, which that one becomes.
    const auto anchor = [&](const uint64_t v, const bool before_side) {
        const VertexState s = state(before_side ? vb : va, v);
        if (ends_chains(s)) return true;
        if (before_side && moves_anchor && v == kept) return false;
        const auto& changed = before_side ? changed_at_before : changed_at_after;
        const auto it = changed.find(v);
        if (it == changed.end() || it->second < s.valence) return true;
        const VertexState o = state(before_side ? va : vb, v);
        return o.alive && o.p == s.p;
    };

    // The chains of one side, each with its ends; kNone ends for a loop through no anchor.
    struct Walked
    {
        Chain chain;
        Segment ends;
    };
    const auto chains_of = [&](const std::set<uint64_t>& segs,
                               const std::map<uint64_t, Geometry>& geom,
                               const bool before_side) {
        std::map<uint64_t, std::vector<uint64_t>> at;
        for (const uint64_t f : segs) {
            for (const uint64_t v : geom.at(f).corners) at[v].push_back(f);
        }
        std::set<uint64_t> done;
        std::vector<Walked> out;
        const auto walk = [&](const uint64_t from, uint64_t f, const bool looped) {
            Walked w;
            uint64_t cur = from;
            while (true) {
                done.insert(f);
                const Geometry& g = geom.at(f);
                const bool fwd = g.corners[0] == cur;
                const double len = (g.p[1] - g.p[0]).norm();
                w.chain.segments.push_back(f);
                w.chain.forward.push_back(fwd);
                w.chain.start.push_back(w.chain.total);
                w.chain.length.push_back(len);
                w.chain.total += len;
                cur = fwd ? g.corners[1] : g.corners[0];
                if (cur == from || anchor(cur, before_side)) break;
                uint64_t next = kNone;
                for (const uint64_t x : at[cur]) {
                    if (!done.count(x)) {
                        next = x;
                        break;
                    }
                }
                if (next == kNone) break;
                f = next;
            }
            measure(w.chain);
            w.ends = looped ? Segment{{kNone, kNone}} : Segment{{from, cur}};
            out.push_back(std::move(w));
        };
        for (const auto& [v, fs] : at) {
            if (!anchor(v, before_side)) continue;
            for (const uint64_t f : fs) {
                if (!done.count(f)) walk(v, f, false);
            }
        }
        for (const uint64_t f : segs) {
            if (!done.count(f)) walk(geom.at(f).corners[0], f, true);
        }
        return out;
    };
    std::vector<Walked> cb = chains_of(changed_before, before, true);
    std::vector<Walked> ca = chains_of(changed_after, after, false);

    // Each chain from its smaller end, the ends before named as after the operation.
    const auto reverse = [](Chain& c) {
        std::reverse(c.segments.begin(), c.segments.end());
        std::reverse(c.forward.begin(), c.forward.end());
        std::reverse(c.length.begin(), c.length.end());
        for (auto&& f : c.forward) f = !f;
        double s = 0;
        for (size_t k = 0; k < c.length.size(); ++k) {
            c.start[k] = s;
            s += c.length[k];
        }
    };
    for (Walked& w : cb) {
        for (uint64_t& x : w.ends) x = x == kNone ? kNone : renamed(x);
    }
    for (std::vector<Walked>* side : {&cb, &ca}) {
        for (Walked& w : *side) {
            if (w.ends[0] > w.ends[1]) {
                reverse(w.chain);
                std::swap(w.ends[0], w.ends[1]);
            }
        }
    }

    // Pairs: by their ends, where those are two vertices only one chain on each side joins;
    // otherwise by a segment both hold, which also says which way round.
    std::vector<int> pair_of(cb.size(), -1);
    std::vector<uint8_t> taken(ca.size(), 0);
    std::map<Segment, std::vector<size_t>> ends_b, ends_a;
    for (size_t k = 0; k < cb.size(); ++k) {
        if (cb[k].ends[0] != kNone && cb[k].ends[0] != cb[k].ends[1])
            ends_b[cb[k].ends].push_back(k);
    }
    for (size_t k = 0; k < ca.size(); ++k) {
        if (ca[k].ends[0] != kNone && ca[k].ends[0] != ca[k].ends[1])
            ends_a[ca[k].ends].push_back(k);
    }
    for (const auto& [ends, bs] : ends_b) {
        const auto it = ends_a.find(ends);
        if (it == ends_a.end() || bs.size() != 1 || it->second.size() != 1) continue;
        pair_of[bs[0]] = int(it->second[0]);
        taken[it->second[0]] = 1;
    }
    for (size_t k = 0; k < cb.size(); ++k) {
        if (pair_of[k] >= 0) continue;
        for (size_t j = 0; j < ca.size() && pair_of[k] < 0; ++j) {
            if (taken[j] || cb[k].ends != ca[j].ends) continue;
            const Chain& b = cb[k].chain;
            for (size_t x = 0; x < b.segments.size() && pair_of[k] < 0; ++x) {
                const auto& a = ca[j].chain.segments;
                const auto it = std::find(a.begin(), a.end(), b.segments[x]);
                if (it == a.end()) continue;
                if (ca[j].chain.forward[size_t(it - a.begin())] != b.forward[x]) {
                    reverse(ca[j].chain);
                }
                pair_of[k] = int(j);
                taken[j] = 1;
            }
        }
    }

    for (size_t k = 0; k < cb.size(); ++k) {
        if (pair_of[k] < 0) continue;
        const uint32_t c = uint32_t(step.chains.size());
        step.chains.push_back({std::move(cb[k].chain), std::move(ca[size_t(pair_of[k])].chain)});
        const auto& [b, a] = step.chains.back();
        for (size_t x = 0; x < b.segments.size(); ++x)
            step.before_at.push_back({b.segments[x], {c, uint32_t(x)}});
        for (size_t x = 0; x < a.segments.size(); ++x)
            step.after_at.push_back({a.segments[x], {c, uint32_t(x)}});
    }

    // The rest changed the topology: each segment onto the one with its corners, if any.
    bool unpaired = false;
    const auto same = [](const Segment& x, const Segment& y) {
        return (x[0] == y[0] && x[1] == y[1]) || (x[0] == y[1] && x[1] == y[0]);
    };
    for (size_t k = 0; k < cb.size(); ++k) {
        if (pair_of[k] >= 0) continue;
        unpaired = true;
        Chain& b = cb[k].chain;
        std::vector<Image> images;
        // The collapsed segment, whose corners are now one: the survivor.
        size_t collapsed = b.segments.size();
        for (size_t x = 0; x < b.segments.size(); ++x) {
            const uint64_t f = b.segments[x];
            const Segment c = {
                {renamed(before.at(f).corners[0]), renamed(before.at(f).corners[1])}};
            Image im{kNone, false};
            const auto it = after.find(f);
            if (it != after.end() && same(it->second.corners, c)) {
                im = {f, it->second.corners[0] != c[0]};
            } else {
                for (const auto& [g, geom] : after) {
                    if (!same(geom.corners, c)) continue;
                    im = {g, geom.corners[0] != c[0]};
                    ++m_build_warnings.merged_segments;
                    break;
                }
            }
            if (c[0] == c[1]) collapsed = x;
            images.push_back(im);
        }
        // The collapsed segment has no image by its corners: they are one now, the survivor. On a
        // chain with a partner it maps by arc length as the rest does. A chain without one that
        // holds it is one the collapse contracted -- two anchors collapsed into one, the segment
        // between them shrunk to the survivor; or a loop of three closed, the rest of the chain
        // merged into the loop's third segment -- and it maps by arc length onto what it became:
        // its other segments' images, end to end, or a chain of no length on a segment at the
        // survivor, where every point goes. A survivor left with no segment has nowhere to put
        // them.
        Chain a;
        bool contracted = collapsed < b.segments.size();
        for (size_t x = 0; x < b.segments.size() && contracted; ++x) {
            if (x == collapsed) continue;
            contracted = images[x].segment != kNone;
            if (!contracted) break;
            const Geometry& g = after.at(images[x].segment);
            a.segments.push_back(images[x].segment);
            a.forward.push_back(bool(b.forward[x]) != images[x].flipped);
            a.length.push_back((g.p[1] - g.p[0]).norm());
        }
        if (contracted && !a.segments.empty()) measure(a);
        if (contracted && a.segments.empty()) {
            for (const auto& [g, geom] : after) {
                if (geom.corners[0] != kept && geom.corners[1] != kept) continue;
                a.segments = {g};
                a.forward = {geom.corners[0] == kept};
                a.start = {0};
                a.length = {0};
                break;
            }
        }
        if (!contracted || a.segments.empty()) {
            for (size_t x = 0; x < b.segments.size(); ++x)
                step.forward.push_back({b.segments[x], images[x]});
            continue;
        }
        // Mapped forward only: what it became maps back as it did, onto its own preimage.
        const uint32_t c = uint32_t(step.chains.size());
        for (size_t x = 0; x < b.segments.size(); ++x)
            step.before_at.push_back({b.segments[x], {c, uint32_t(x)}});
        step.chains.push_back({std::move(b), std::move(a)});
    }
    for (size_t j = 0; j < ca.size(); ++j) {
        if (taken[j]) continue;
        unpaired = true;
        for (const uint64_t g : ca[j].chain.segments) {
            const Segment& c = after.at(g).corners;
            Image im{kNone, false};
            const auto it = before.find(g);
            const auto renamed_of = [&](const Geometry& x) {
                return Segment{{renamed(x.corners[0]), renamed(x.corners[1])}};
            };
            if (it != before.end() && same(renamed_of(it->second), c)) {
                im = {g, renamed_of(it->second)[0] != c[0]};
            } else {
                for (const auto& [f, geom] : before) {
                    if (!same(renamed_of(geom), c)) continue;
                    im = {f, renamed_of(geom)[0] != c[0]};
                    break;
                }
            }
            step.backward.push_back({g, im});
        }
    }
    if (unpaired || welded) ++m_build_warnings.topology;

    const auto by_segment = [](const auto& x, const auto& y) { return x.first < y.first; };
    std::sort(step.before_at.begin(), step.before_at.end(), by_segment);
    std::sort(step.after_at.begin(), step.after_at.end(), by_segment);
    std::sort(step.forward.begin(), step.forward.end(), by_segment);
    std::sort(step.backward.begin(), step.backward.end(), by_segment);
}

void CurveMap::build_renumbering(CurveReplay& r, const Edit& e, Step& step)
{
    const std::vector<Segment> old = r.faces();
    const std::vector<uint8_t> alive = r.face_alive();
    r.apply(e);
    ++m_steps_by_kind[size_t(e.kind)];
    const bool relabel = e.kind == EditKind::relabel;
    step.old_to_new.assign(old.size(), {kNone, false});
    step.new_to_old.assign(r.faces().size(), {kNone, false});
    uint64_t rank = 0;
    for (uint64_t f = 0; f < old.size(); ++f) {
        if (!alive[f]) continue;
        Segment c = old[f];
        if (relabel) {
            for (uint64_t& v : c) v = v < e.v.size() ? e.v[v] : kNone;
            if (c[0] == kNone || c[1] == kNone || c[0] == c[1]) continue; // a weld took it
        }
        uint64_t g = rank++;
        if (relabel && !e.faces.empty()) g = f < e.faces.size() ? e.faces[f] : kNone;
        if (g >= r.faces().size()) continue;
        // A consolidation keeps every corner order; a relabel may give a duplicate's.
        const bool flipped = relabel && r.faces()[g][0] != c[0];
        step.old_to_new[f] = {g, flipped};
        if (step.new_to_old[g].segment == kNone) {
            step.new_to_old[g] = {f, flipped};
        } else {
            ++m_build_warnings.merged_segments;
        }
    }
}

void CurveMap::build_embed(CurveReplay& r, const Edit& e, Step& step)
{
    const auto positions = [&r]() {
        std::vector<std::array<Eigen::Vector3d, 2>> out(r.faces().size());
        for (uint64_t f = 0; f < r.faces().size(); ++f) {
            if (r.face_alive()[f]) out[f] = geometry(r, f).p;
        }
        return out;
    };
    step.before = positions();
    const std::vector<uint8_t> alive = r.face_alive();
    r.apply(e);
    ++m_steps_by_kind[size_t(e.kind)];
    step.after = positions();
    const size_t nf = e.v.size() / 2;
    step.to_before.assign(nf, {});
    step.to_after.assign(step.before.size(), {});
    size_t i = 0;
    for (uint64_t g = 0; g < nf; ++g) {
        const uint64_t n = e.faces[i++];
        for (uint64_t k = 0; k < n; ++k, ++i) {
            const uint64_t f = e.faces[i];
            if (f >= alive.size() || !alive[f]) continue;
            step.to_before[g].push_back(f);
            step.to_after[f].push_back(g);
        }
        if (step.to_before[g].size() > 1) ++m_build_warnings.overlapping;
    }

    // Each segment before and the segments after on it, end to end from its first corner: a
    // chain pair, mapped by arc length like a local map. On collinear pieces that is where a
    // point is in space; it also tells apart pieces of no length, which a position cannot.
    for (uint64_t f = 0; f < step.to_after.size(); ++f) {
        const auto& gs = step.to_after[f];
        if (gs.empty()) continue;
        std::map<uint64_t, std::vector<uint64_t>> at;
        for (const uint64_t g : gs) {
            for (const uint64_t v : r.faces()[g]) at[v].push_back(g);
        }
        uint64_t from = kNone;
        double nearest = std::numeric_limits<double>::infinity();
        size_t n_ends = 0;
        for (const auto& [v, segs] : at) {
            if (segs.size() > 2) n_ends = 3; // not a path
            if (segs.size() != 1) continue;
            ++n_ends;
            const double d = (r.vertices()[v] - step.before[f][0]).squaredNorm();
            if (d < nearest) {
                nearest = d;
                from = v;
            }
        }
        if (n_ends != 2) continue; // projected instead
        Chain a, b;
        std::set<uint64_t> done;
        uint64_t cur = from, g = at[from].front();
        while (true) {
            done.insert(g);
            const Segment& c = r.faces()[g];
            const bool fwd = c[0] == cur;
            a.segments.push_back(g);
            a.forward.push_back(fwd);
            a.length.push_back((step.after[g][1] - step.after[g][0]).norm());
            cur = fwd ? c[1] : c[0];
            uint64_t next = kNone;
            for (const uint64_t x : at[cur]) {
                if (!done.count(x)) next = x;
            }
            if (next == kNone) break;
            g = next;
        }
        if (done.size() != gs.size()) continue;
        b.segments = {f};
        b.forward = {1};
        b.length = {(step.before[f][1] - step.before[f][0]).norm()};
        measure(a);
        measure(b);
        const uint32_t k = uint32_t(step.chains.size());
        step.before_at.push_back({f, {k, 0}});
        for (size_t x = 0; x < a.segments.size(); ++x) {
            step.after_at.push_back({a.segments[x], {k, uint32_t(x)}});
        }
        step.chains.push_back({std::move(b), std::move(a)});
    }
    // A segment after on more than one before maps back onto the first.
    const auto by_segment = [](const auto& x, const auto& y) { return x.first < y.first; };
    std::stable_sort(step.after_at.begin(), step.after_at.end(), by_segment);
    step.after_at.erase(
        std::unique(
            step.after_at.begin(),
            step.after_at.end(),
            [](const auto& x, const auto& y) { return x.first == y.first; }),
        step.after_at.end());
}

void CurveMap::measure(Chain& c)
{
    const size_t n = c.length.size();
    double total = 0;
    for (const double l : c.length) total += l;
    const double floor = total > 0 ? 1e-9 * total / double(n) : 0;
    c.start.assign(n, 0);
    c.total = 0;
    for (size_t k = 0; k < n; ++k) {
        c.length[k] = total > 0 ? std::max(c.length[k], floor) : 1;
        c.start[k] = c.total;
        c.total += c.length[k];
    }
}

void CurveMap::build_replace(CurveReplay& r, const Edit& e, Step& step)
{
    const size_t n_added = e.v.size() / 2, n_removed = e.faces.size() - n_added;
    for (size_t k = 0; k < n_removed; ++k) step.forward.push_back({e.faces[k], {kNone, false}});
    for (size_t k = 0; k < n_added; ++k) {
        step.backward.push_back({e.faces[n_removed + k], {kNone, false}});
    }
    const auto by_segment = [](const auto& x, const auto& y) { return x.first < y.first; };
    std::sort(step.forward.begin(), step.forward.end(), by_segment);
    std::sort(step.backward.begin(), step.backward.end(), by_segment);
    r.apply(e);
    ++m_steps_by_kind[size_t(e.kind)];
    m_build_warnings.replaced += n_removed + n_added;
}

CurvePoint
CurveMap::along(const Chain& from, const size_t i, const Eigen::Vector2d& bary, const Chain& to)
{
    const double u = from.forward[i] ? bary[1] : bary[0];
    const double s = from.total > 0 ? (from.start[i] + u * from.length[i]) / from.total : 0;
    const double x = s * to.total;
    // The last segment starting at or before x -- of positive length, if there is one there.
    size_t j = size_t(std::upper_bound(to.start.begin(), to.start.end(), x) - to.start.begin());
    j = j == 0 ? 0 : j - 1;
    while (j > 0 && !(to.length[j] > 0)) --j;
    const double v = to.length[j] > 0 ? std::clamp((x - to.start[j]) / to.length[j], 0., 1.) : 0;
    return {to.segments[j], to.forward[j] ? bary_of(v) : bary_of(1 - v)};
}

bool CurveMap::forward(const Step& s, CurvePoint& p)
{
    switch (s.kind) {
    case EditKind::consolidate:
    case EditKind::relabel: {
        if (p.segment >= s.old_to_new.size()) return false;
        const Image& im = s.old_to_new[p.segment];
        if (im.segment == kNone) return false;
        p.segment = im.segment;
        if (im.flipped) std::swap(p.bary[0], p.bary[1]);
        return true;
    }
    case EditKind::embed: {
        if (const Place* at = find_in(s.before_at, p.segment)) {
            const auto& [b, a] = s.chains[at->first];
            p = along(b, at->second, p.bary, a);
            return true;
        }
        if (p.segment >= s.to_after.size() || s.to_after[p.segment].empty()) return false;
        const auto& b = s.before[p.segment];
        const Eigen::Vector3d x = p.bary[0] * b[0] + p.bary[1] * b[1];
        double best = std::numeric_limits<double>::infinity();
        for (const uint64_t g : s.to_after[p.segment]) {
            const auto& a = s.after[g];
            const double t = parameter(a[0], a[1], x);
            const double d = (a[0] + t * (a[1] - a[0]) - x).squaredNorm();
            if (d < best) {
                best = d;
                p = {g, bary_of(t)};
            }
        }
        return true;
    }
    default: {
        if (const Place* at = find_in(s.before_at, p.segment)) {
            const auto& [b, a] = s.chains[at->first];
            p = along(b, at->second, p.bary, a);
            return true;
        }
        if (const Image* im = find_in(s.forward, p.segment)) {
            if (im->segment == kNone) return false;
            p.segment = im->segment;
            if (im->flipped) std::swap(p.bary[0], p.bary[1]);
        }
        return true;
    }
    }
}

bool CurveMap::backward(const Step& s, CurvePoint& p)
{
    switch (s.kind) {
    case EditKind::consolidate:
    case EditKind::relabel: {
        if (p.segment >= s.new_to_old.size()) return false;
        const Image& im = s.new_to_old[p.segment];
        if (im.segment == kNone) return false;
        p.segment = im.segment;
        if (im.flipped) std::swap(p.bary[0], p.bary[1]);
        return true;
    }
    case EditKind::embed: {
        if (const Place* at = find_in(s.after_at, p.segment)) {
            const auto& [b, a] = s.chains[at->first];
            p = along(a, at->second, p.bary, b);
            return true;
        }
        if (p.segment >= s.to_before.size() || s.to_before[p.segment].empty()) return false;
        const auto& a = s.after[p.segment];
        const Eigen::Vector3d y = p.bary[0] * a[0] + p.bary[1] * a[1];
        const uint64_t f = s.to_before[p.segment].front();
        const auto& b = s.before[f];
        p = {f, bary_of(parameter(b[0], b[1], y))};
        return true;
    }
    default: {
        if (const Place* at = find_in(s.after_at, p.segment)) {
            const auto& [b, a] = s.chains[at->first];
            p = along(a, at->second, p.bary, b);
            return true;
        }
        if (const Image* im = find_in(s.backward, p.segment)) {
            if (im->segment == kNone) return false;
            p.segment = im->segment;
            if (im->flipped) std::swap(p.bary[0], p.bary[1]);
        }
        return true;
    }
    }
}

void CurveMap::map(
    std::vector<CurvePoint>& points,
    const bool to_output,
    CurveMapWarnings& warnings) const
{
    const auto& alive = to_output ? m_in_alive : m_out_alive;
    // The points on each segment.
    std::unordered_map<uint64_t, std::vector<uint32_t>> on;
    const auto index = [&]() {
        on.clear();
        for (uint32_t i = 0; i < points.size(); ++i) {
            if (points[i].segment != kNone) on[points[i].segment].push_back(i);
        }
    };
    for (CurvePoint& p : points) {
        if (p.segment >= alive.size() || !alive[p.segment]) {
            p.segment = kNone;
            ++warnings.unlocated;
        }
    }
    index();
    const size_t n = m_steps.size();
    for (size_t k = 0; k < n; ++k) {
        const Step& s = m_steps[to_output ? k : n - 1 - k];
        const auto move = [&](CurvePoint& p) {
            if (to_output ? forward(s, p) : backward(s, p)) return;
            p.segment = kNone;
            ++warnings.unlocated;
        };
        if (s.kind == EditKind::consolidate || s.kind == EditKind::relabel ||
            s.kind == EditKind::embed) {
            for (CurvePoint& p : points) {
                if (p.segment != kNone) move(p);
            }
            index();
            continue;
        }
        // The segments this step maps; the points on them, taken out, mapped, put back.
        std::vector<uint32_t> moved;
        const auto take = [&](const uint64_t f) {
            const auto it = on.find(f);
            if (it == on.end()) return;
            moved.insert(moved.end(), it->second.begin(), it->second.end());
            on.erase(it);
        };
        for (const auto& [f, at] : to_output ? s.before_at : s.after_at) take(f);
        for (const auto& [f, im] : to_output ? s.forward : s.backward) take(f);
        for (const uint32_t i : moved) {
            move(points[i]);
            if (points[i].segment != kNone) on[points[i].segment].push_back(i);
        }
    }
}

std::vector<CurvePoint> CurveMap::to_output(
    std::vector<CurvePoint> points,
    CurveMapWarnings* warnings) const
{
    CurveMapWarnings w;
    map(points, true, w);
    if (warnings) *warnings = w;
    return points;
}

std::vector<CurvePoint> CurveMap::to_input(
    std::vector<CurvePoint> points,
    CurveMapWarnings* warnings) const
{
    CurveMapWarnings w;
    map(points, false, w);
    if (warnings) *warnings = w;
    return points;
}

} // namespace wmtk::components::tracking
