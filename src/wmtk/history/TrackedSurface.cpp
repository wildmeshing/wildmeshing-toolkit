#include "TrackedSurface.hpp"

#include <algorithm>
#include <iterator>
#include <map>

namespace wmtk::history {

namespace {

using Triangle = TrackedSurfaceRecorder::Triangle;
constexpr uint64_t kNoSurface = uint64_t(-1);
constexpr size_t kNoMesh = size_t(-1);

Triangle sorted(Triangle t)
{
    std::sort(t.begin(), t.end());
    return t;
}

/// Sorted triples, sorted, without duplicates: a set of faces.
std::vector<Triangle> as_set(const std::vector<Triangle>& ts)
{
    std::vector<Triangle> out;
    out.reserve(ts.size());
    for (const Triangle& t : ts) out.push_back(sorted(t));
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::vector<Triangle> minus(const std::vector<Triangle>& a, const std::vector<Triangle>& b)
{
    std::vector<Triangle> out;
    std::set_difference(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(out));
    return out;
}

bool has(const Triangle& t, const size_t v)
{
    return t[0] == v || t[1] == v || t[2] == v;
}

Triangle renamed(Triangle t, const size_t from, const size_t to)
{
    for (size_t& v : t) {
        if (v == from) v = to;
    }
    return sorted(t);
}

SurfaceHistory snapshot(
    const std::vector<Triangle>& faces,
    const TrackedSurfaceRecorder::Positions& position,
    const TrackedSurfaceRecorder::Orient& orient,
    std::vector<uint64_t>& surface_of,
    std::vector<size_t>& mesh_of,
    const std::vector<std::array<size_t, 2>>& feature_edges)
{
    SurfaceHistory h;
    for (const Triangle& f : as_set(faces)) {
        const Triangle o = orient(f);
        std::array<uint64_t, 3> t;
        for (int k = 0; k < 3; ++k) {
            const size_t v = o[k];
            if (v >= surface_of.size()) surface_of.resize(v + 1, kNoSurface);
            if (surface_of[v] == kNoSurface) {
                surface_of[v] = mesh_of.size();
                mesh_of.push_back(v);
                h.vertices.push_back(position(v));
            }
            t[k] = surface_of[v];
        }
        h.faces.push_back(t);
    }
    h.vertex_alive.assign(h.vertices.size(), 1);
    h.face_alive.assign(h.faces.size(), 1);
    for (const auto& e : feature_edges) {
        if (e[0] >= surface_of.size() || e[1] >= surface_of.size()) continue;
        uint64_t a = surface_of[e[0]], b = surface_of[e[1]];
        if (a == kNoSurface || b == kNoSurface) continue;
        if (a > b) std::swap(a, b);
        h.feature_edges.push_back({{a, b}});
    }
    std::sort(h.feature_edges.begin(), h.feature_edges.end());
    h.feature_edges.erase(
        std::unique(h.feature_edges.begin(), h.feature_edges.end()),
        h.feature_edges.end());
    return h;
}

} // namespace

TrackedSurfaceRecorder::TrackedSurfaceRecorder(
    const std::vector<Triangle>& faces,
    Positions position,
    Orient orient,
    const std::vector<std::array<size_t, 2>>& feature_edges)
    : m_position(std::move(position))
    , m_orient(std::move(orient))
    , m_history(snapshot(faces, m_position, m_orient, m_surface_of, m_mesh_of, feature_edges))
    , m_replay(m_history)
    , m_known(m_history.vertices)
    , m_next_face(m_history.faces.size())
{}

std::vector<uint64_t> TrackedSurfaceRecorder::mesh_vertex_ids() const
{
    std::vector<uint64_t> out(m_mesh_of.size(), kNoVertex);
    for (size_t s = 0; s < m_mesh_of.size(); ++s) {
        if (m_mesh_of[s] != kNoMesh && live(m_mesh_of[s]) == s) out[s] = m_mesh_of[s];
    }
    return out;
}

std::vector<std::array<size_t, 2>> TrackedSurfaceRecorder::feature_edges() const
{
    std::vector<std::array<size_t, 2>> out;
    for (const auto& e : m_replay.feature_edges()) {
        std::array<size_t, 2> m = {{m_mesh_of[e[0]], m_mesh_of[e[1]]}};
        if (m[0] > m[1]) std::swap(m[0], m[1]);
        out.push_back(m);
    }
    std::sort(out.begin(), out.end());
    return out;
}

std::optional<uint64_t> TrackedSurfaceRecorder::live(const size_t v) const
{
    if (v >= m_surface_of.size() || m_surface_of[v] == kNoSurface) return std::nullopt;
    const uint64_t s = m_surface_of[v];
    if (s >= m_replay.vertex_alive().size() || !m_replay.vertex_alive()[s]) return std::nullopt;
    return s;
}

std::optional<uint64_t> TrackedSurfaceRecorder::lookup(const Triangle& t) const
{
    std::array<uint64_t, 3> s;
    for (int k = 0; k < 3; ++k) {
        const auto sk = live(t[k]);
        if (!sk) return std::nullopt;
        s[k] = *sk;
    }
    const auto& f1 = m_replay.faces_of(s[1]);
    const auto& f2 = m_replay.faces_of(s[2]);
    for (const uint64_t f : m_replay.faces_of(s[0])) {
        if (std::binary_search(f1.begin(), f1.end(), f) &&
            std::binary_search(f2.begin(), f2.end(), f)) {
            return f;
        }
    }
    return std::nullopt;
}

uint64_t TrackedSurfaceRecorder::fresh_vertex(const size_t v)
{
    if (v >= m_surface_of.size()) m_surface_of.resize(v + 1, kNoSurface);
    if (m_surface_of[v] != kNoSurface) m_mesh_of[m_surface_of[v]] = kNoMesh;
    const uint64_t s = m_mesh_of.size();
    m_mesh_of.push_back(v);
    m_surface_of[v] = s;
    m_known.resize(m_mesh_of.size(), Eigen::Vector3d::Zero());
    return s;
}

void TrackedSurfaceRecorder::record(
    Edit&& e,
    const std::vector<size_t>& written,
    const std::vector<uint64_t>& fresh)
{
    m_replay.apply(e); // the topology: `e` has no moves yet
    for (const uint64_t s : fresh) {
        const Eigen::Vector3d p = m_position(m_mesh_of[s]);
        m_known[s] = p;
        e.moves.push_back({s, p});
    }
    for (const size_t v : written) {
        const auto s = live(v);
        if (!s || std::find(fresh.begin(), fresh.end(), *s) != fresh.end()) continue;
        const Eigen::Vector3d p = m_position(v);
        if (m_known[*s] == p) continue;
        m_known[*s] = p;
        e.moves.push_back({*s, p});
    }
    if (!e.moves.empty()) m_replay.apply(Edit{EditKind::move_vertices, {}, {}, e.moves});
    if (e.kind == EditKind::move_vertices && e.moves.empty()) return;
    m_history.edits.push_back(std::move(e));
}

std::optional<Edit> TrackedSurfaceRecorder::as_split(
    const Change& c,
    const std::vector<Triangle>& before,
    const std::vector<Triangle>& removed,
    const std::vector<Triangle>& added,
    std::vector<uint64_t>& fresh)
{
    const size_t v0 = c.v[0], v1 = c.v[1], vn = c.v[2];
    const auto s0 = live(v0), s1 = live(v1);
    if (!s0 || !s1 || live(vn)) return std::nullopt;
    // Every face on the edge goes, and its two halves come.
    std::vector<Triangle> on_edge, halves;
    for (const Triangle& t : before) {
        if (!has(t, v0) || !has(t, v1)) continue;
        on_edge.push_back(t);
        halves.push_back(renamed(t, v1, vn));
        halves.push_back(renamed(t, v0, vn));
    }
    if (on_edge != removed || as_set(halves) != added) return std::nullopt;
    std::vector<uint64_t> fs;
    for (const Triangle& t : on_edge) {
        const auto f = lookup(t);
        if (!f) return std::nullopt;
        fs.push_back(*f);
    }
    const uint64_t sn = fresh_vertex(vn);
    fresh.push_back(sn);
    Edit e{EditKind::split_edge, {*s0, *s1, sn}, {}, {}};
    for (const uint64_t f : fs) {
        e.faces.push_back(f);
        e.faces.push_back(m_next_face++);
    }
    ++m_stats.split;
    return e;
}

std::optional<Edit> TrackedSurfaceRecorder::as_collapse(
    Change& c,
    const std::vector<Triangle>& before,
    const std::vector<Triangle>& after)
{
    const size_t r = c.v[0], k = c.v[1];
    const auto sr = live(r);
    if (!sr) return std::nullopt;
    // The faces at `removed` are renamed to `kept`, those with both go, duplicates merge.
    std::vector<Triangle> predicted;
    bool spanning = false;
    for (const Triangle& t : before) {
        if (has(t, r) && has(t, k)) {
            spanning = true;
            continue;
        }
        if (has(t, r) && !lookup(t)) return std::nullopt;
        predicted.push_back(renamed(t, r, k));
    }
    if (as_set(predicted) != after) return std::nullopt;
    ++m_stats.collapse;
    const auto sk = live(k);
    m_surface_of[r] = kNoSurface;
    if (sk) {
        if (!spanning) ++m_stats.weld;
        m_mesh_of[*sr] = kNoMesh;
        return Edit{EditKind::collapse_edge, {*sr, *sk}, {}, {}};
    }
    // `kept` was not on the surface: it takes the removed vertex's place there, which moves.
    if (k >= m_surface_of.size()) m_surface_of.resize(k + 1, kNoSurface);
    if (m_surface_of[k] != kNoSurface) m_mesh_of[m_surface_of[k]] = kNoMesh; // retired before
    m_surface_of[k] = *sr;
    m_mesh_of[*sr] = k;
    c.written.push_back(k);
    return Edit{EditKind::move_vertices, {}, {}, {}};
}

std::optional<Edit> TrackedSurfaceRecorder::as_swap(
    const std::vector<Triangle>& removed,
    const std::vector<Triangle>& added)
{
    if (removed.size() != 2 || added.size() != 2) return std::nullopt;
    std::vector<size_t> edge, apexes;
    for (const size_t v : removed[0]) (has(removed[1], v) ? edge : apexes).push_back(v);
    for (const size_t v : removed[1]) {
        if (!has(removed[0], v)) apexes.push_back(v);
    }
    if (edge.size() != 2 || apexes.size() != 2) return std::nullopt;
    const size_t a = edge[0], b = edge[1], c = apexes[0], d = apexes[1];
    if (as_set({{{a, c, d}}, {{b, c, d}}}) != added) return std::nullopt;
    const auto sa = live(a), sb = live(b), sc = live(c), sd = live(d);
    const auto f_abc = lookup(removed[0]), f_abd = lookup(removed[1]);
    if (!sa || !sb || !sc || !sd || !f_abc || !f_abd) return std::nullopt;
    ++m_stats.swap;
    // (a, b) becomes (c, d): in the face with apex d, b is renamed to c; in the one with apex c,
    // a is renamed to d. Each keeps its corner order, so the two stay oriented alike.
    return Edit{EditKind::swap_edge, {*sa, *sb, *sc, *sd}, {*f_abd, *f_abc}, {}};
}

Edit TrackedSurfaceRecorder::replacement(
    const std::vector<Triangle>& removed,
    const std::vector<Triangle>& added,
    std::vector<uint64_t>& fresh)
{
    Edit e{EditKind::replace_faces, {}, {}, {}};
    for (const Triangle& t : removed) {
        if (const auto f = lookup(t)) {
            e.faces.push_back(*f);
        } else {
            ++m_stats.unknown;
        }
    }
    std::map<size_t, uint64_t> assigned;
    for (const Triangle& t : added) {
        for (const size_t v : m_orient(t)) {
            uint64_t s;
            if (const auto it = assigned.find(v); it != assigned.end()) {
                s = it->second;
            } else if (const auto l = live(v)) {
                s = *l;
            } else {
                s = fresh_vertex(v);
                fresh.push_back(s);
            }
            assigned[v] = s;
            e.v.push_back(s);
        }
        e.faces.push_back(m_next_face++);
    }
    return e;
}

void TrackedSurfaceRecorder::commit(Change&& c)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    const std::vector<Triangle> before = as_set(c.before), after = as_set(c.after);
    const std::vector<Triangle> removed = minus(before, after), added = minus(after, before);
    std::sort(c.written.begin(), c.written.end());
    c.written.erase(std::unique(c.written.begin(), c.written.end()), c.written.end());
    std::vector<uint64_t> fresh;
    if (removed.empty() && added.empty()) {
        const size_t edits = m_history.edits.size();
        record(Edit{EditKind::move_vertices, {}, {}, {}}, c.written, fresh);
        if (m_history.edits.size() > edits) ++m_stats.move;
        return;
    }
    std::optional<Edit> e;
    switch (c.kind) {
    case EditKind::split_edge: e = as_split(c, before, removed, added, fresh); break;
    case EditKind::collapse_edge: e = as_collapse(c, before, after); break;
    case EditKind::swap_edge: e = as_swap(removed, added); break;
    default: break;
    }
    if (!e) {
        e = replacement(removed, added, fresh);
        ++m_stats.replaced;
    }
    record(std::move(*e), c.written, fresh);
}

void TrackedSurfaceRecorder::renumber(const std::vector<size_t>& old_to_new)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<uint64_t> next;
    for (size_t v = 0; v < m_surface_of.size(); ++v) {
        const uint64_t s = m_surface_of[v];
        if (s == kNoSurface) continue;
        const size_t n = v < old_to_new.size() ? old_to_new[v] : kNoMesh;
        m_mesh_of[s] = n;
        if (n == kNoMesh) continue;
        if (n >= next.size()) next.resize(n + 1, kNoSurface);
        next[n] = s;
    }
    m_surface_of = std::move(next);
}

size_t TrackedSurfaceRecorder::resync(const std::vector<Triangle>& faces)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<Triangle> current;
    for (uint64_t f = 0; f < m_replay.faces().size(); ++f) {
        if (!m_replay.face_alive()[f]) continue;
        Triangle t;
        for (int k = 0; k < 3; ++k) t[k] = m_mesh_of[m_replay.faces()[f][k]];
        current.push_back(t);
    }
    current = as_set(current);
    const std::vector<Triangle> target = as_set(faces);
    const std::vector<Triangle> removed = minus(current, target), added = minus(target, current);
    if (removed.empty() && added.empty()) return 0;
    std::vector<uint64_t> fresh;
    record(replacement(removed, added, fresh), {}, fresh);
    ++m_stats.replaced;
    return removed.size() + added.size();
}

TrackedSurfaceRecorder::Stats TrackedSurfaceRecorder::stats() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_stats;
}

std::optional<uint64_t> TrackedSurfaceRecorder::face_id(const Triangle& t) const
{
    return lookup(t);
}

std::optional<uint64_t> TrackedSurfaceRecorder::vertex_id(const size_t v) const
{
    return live(v);
}

} // namespace wmtk::history
