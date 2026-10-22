#include "TrackedRecorder.hpp"

#include <algorithm>
#include <iterator>
#include <map>

namespace wmtk::history {

namespace {

constexpr uint64_t kNoComplex = uint64_t(-1);
constexpr size_t kNoMesh = size_t(-1);

template <size_t K>
std::array<size_t, K> sorted(std::array<size_t, K> t)
{
    std::sort(t.begin(), t.end());
    return t;
}

/// Sorted cells, sorted, without duplicates: a set of faces.
template <size_t K>
std::vector<std::array<size_t, K>> as_set(const std::vector<std::array<size_t, K>>& ts)
{
    std::vector<std::array<size_t, K>> out;
    out.reserve(ts.size());
    for (const auto& t : ts) out.push_back(sorted(t));
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

template <size_t K>
std::vector<std::array<size_t, K>> minus(
    const std::vector<std::array<size_t, K>>& a,
    const std::vector<std::array<size_t, K>>& b)
{
    std::vector<std::array<size_t, K>> out;
    std::set_difference(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(out));
    return out;
}

template <size_t K>
bool has(const std::array<size_t, K>& t, const size_t v)
{
    return std::find(t.begin(), t.end(), v) != t.end();
}

template <size_t K>
std::array<size_t, K> renamed(std::array<size_t, K> t, const size_t from, const size_t to)
{
    for (size_t& v : t) {
        if (v == from) v = to;
    }
    return sorted(t);
}

template <int N>
History<N> snapshot(
    const std::vector<typename TrackedRecorder<N>::Cell>& faces,
    const typename TrackedRecorder<N>::Positions& position,
    const typename TrackedRecorder<N>::Orient& orient,
    std::vector<uint64_t>& complex_of,
    std::vector<size_t>& mesh_of,
    const std::vector<typename TrackedRecorder<N>::MeshFeature>& features)
{
    History<N> h;
    const auto vertex = [&](const size_t v) {
        if (v >= complex_of.size()) complex_of.resize(v + 1, kNoComplex);
        if (complex_of[v] == kNoComplex) {
            complex_of[v] = mesh_of.size();
            mesh_of.push_back(v);
            h.vertices.push_back(position(v));
        }
        return complex_of[v];
    };
    for (const auto& f : as_set(faces)) {
        const auto o = orient(f);
        typename History<N>::Face t;
        for (int k = 0; k < N; ++k) t[k] = vertex(o[k]);
        h.faces.push_back(t);
    }
    for (const auto& e : features) {
        typename History<N>::Feature s;
        bool on = true;
        for (int k = 0; k < N - 1; ++k) {
            if (N == 2) {
                // A feature point is a vertex of the complex, on the curves or not.
                s[k] = vertex(e[k]);
                continue;
            }
            on = on && e[k] < complex_of.size() && complex_of[e[k]] != kNoComplex;
            if (on) s[k] = complex_of[e[k]];
        }
        if (!on) continue;
        std::sort(s.begin(), s.end());
        h.features.push_back(s);
    }
    h.vertex_alive.assign(h.vertices.size(), 1);
    h.face_alive.assign(h.faces.size(), 1);
    std::sort(h.features.begin(), h.features.end());
    h.features.erase(std::unique(h.features.begin(), h.features.end()), h.features.end());
    return h;
}

} // namespace

template <int N>
TrackedRecorder<N>::TrackedRecorder(
    const std::vector<Cell>& faces,
    Positions position,
    Orient orient,
    const std::vector<MeshFeature>& features)
    : m_position(std::move(position))
    , m_orient(std::move(orient))
    , m_history(snapshot<N>(faces, m_position, m_orient, m_complex_of, m_mesh_of, features))
    , m_replay(m_history)
    , m_known(m_history.vertices)
    , m_next_face(m_history.faces.size())
{}

template <int N>
std::vector<uint64_t> TrackedRecorder<N>::mesh_vertex_ids() const
{
    std::vector<uint64_t> out(m_mesh_of.size(), kNoVertex);
    for (size_t s = 0; s < m_mesh_of.size(); ++s) {
        if (m_mesh_of[s] != kNoMesh && live(m_mesh_of[s]) == s) out[s] = m_mesh_of[s];
    }
    return out;
}

template <int N>
std::vector<typename TrackedRecorder<N>::MeshFeature> TrackedRecorder<N>::features() const
{
    std::vector<MeshFeature> out;
    for (const auto& e : m_replay.features()) {
        MeshFeature m;
        for (int k = 0; k < N - 1; ++k) m[k] = m_mesh_of[e[k]];
        out.push_back(sorted(m));
    }
    std::sort(out.begin(), out.end());
    return out;
}

template <int N>
std::optional<uint64_t> TrackedRecorder<N>::live(const size_t v) const
{
    if (v >= m_complex_of.size() || m_complex_of[v] == kNoComplex) return std::nullopt;
    const uint64_t s = m_complex_of[v];
    if (s >= m_replay.vertex_alive().size() || !m_replay.vertex_alive()[s]) return std::nullopt;
    return s;
}

template <int N>
std::optional<uint64_t> TrackedRecorder<N>::lookup(const Cell& t) const
{
    std::array<uint64_t, N> s;
    for (int k = 0; k < N; ++k) {
        const auto sk = live(t[k]);
        if (!sk) return std::nullopt;
        s[k] = *sk;
    }
    for (const uint64_t f : m_replay.faces_of(s[0])) {
        bool all = true;
        for (int k = 1; k < N; ++k) {
            const auto& fk = m_replay.faces_of(s[k]);
            all = all && std::binary_search(fk.begin(), fk.end(), f);
        }
        if (all) return f;
    }
    return std::nullopt;
}

template <int N>
uint64_t TrackedRecorder<N>::fresh_vertex(const size_t v)
{
    if (v >= m_complex_of.size()) m_complex_of.resize(v + 1, kNoComplex);
    if (m_complex_of[v] != kNoComplex) m_mesh_of[m_complex_of[v]] = kNoMesh;
    const uint64_t s = m_mesh_of.size();
    m_mesh_of.push_back(v);
    m_complex_of[v] = s;
    m_known.resize(m_mesh_of.size(), Eigen::Vector3d::Zero());
    return s;
}

template <int N>
void TrackedRecorder<N>::record(
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

template <int N>
std::optional<Edit> TrackedRecorder<N>::as_split(
    const Change& c,
    const std::vector<Cell>& before,
    const std::vector<Cell>& removed,
    const std::vector<Cell>& added,
    std::vector<uint64_t>& fresh)
{
    const size_t v0 = c.v[0], v1 = c.v[1], vn = c.v[2];
    const auto s0 = live(v0), s1 = live(v1);
    if (!s0 || !s1 || live(vn)) return std::nullopt;
    // Every face on the edge goes, and its two halves come.
    std::vector<Cell> on_edge, halves;
    for (const Cell& t : before) {
        if (!has(t, v0) || !has(t, v1)) continue;
        on_edge.push_back(t);
        halves.push_back(renamed(t, v1, vn));
        halves.push_back(renamed(t, v0, vn));
    }
    if (on_edge != removed || as_set(halves) != added) return std::nullopt;
    std::vector<uint64_t> fs;
    for (const Cell& t : on_edge) {
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

template <int N>
std::optional<Edit> TrackedRecorder<N>::as_collapse(
    Change& c,
    const std::vector<Cell>& before,
    const std::vector<Cell>& after)
{
    const size_t r = c.v[0], k = c.v[1];
    const auto sr = live(r);
    if (!sr) return std::nullopt;
    // The faces at `removed` are renamed to `kept`, those with both go, duplicates merge.
    std::vector<Cell> predicted;
    bool spanning = false;
    for (const Cell& t : before) {
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
    m_complex_of[r] = kNoComplex;
    if (sk) {
        if (!spanning) ++m_stats.weld;
        m_mesh_of[*sr] = kNoMesh;
        return Edit{EditKind::collapse_edge, {*sr, *sk}, {}, {}};
    }
    // `kept` was not on the surface: it takes the removed vertex's place there, which moves.
    if (k >= m_complex_of.size()) m_complex_of.resize(k + 1, kNoComplex);
    if (m_complex_of[k] != kNoComplex) m_mesh_of[m_complex_of[k]] = kNoMesh; // retired before
    m_complex_of[k] = *sr;
    m_mesh_of[*sr] = k;
    c.written.push_back(k);
    return Edit{EditKind::move_vertices, {}, {}, {}};
}

template <int N>
std::optional<Edit> TrackedRecorder<N>::as_swap(
    const std::vector<Cell>& removed,
    const std::vector<Cell>& added)
{
    if constexpr (N != 3) {
        return std::nullopt; // a swap never takes a tracked edge
    } else {
        if (removed.size() != 2 || added.size() != 2) return std::nullopt;
        std::vector<size_t> edge, apexes;
        for (const size_t v : removed[0]) (has(removed[1], v) ? edge : apexes).push_back(v);
        for (const size_t v : removed[1]) {
            if (!has(removed[0], v)) apexes.push_back(v);
        }
        if (edge.size() != 2 || apexes.size() != 2) return std::nullopt;
        const size_t a = edge[0], b = edge[1], c = apexes[0], d = apexes[1];
        if (as_set(std::vector<Cell>{{{a, c, d}}, {{b, c, d}}}) != added) return std::nullopt;
        const auto sa = live(a), sb = live(b), sc = live(c), sd = live(d);
        const auto f_abc = lookup(removed[0]), f_abd = lookup(removed[1]);
        if (!sa || !sb || !sc || !sd || !f_abc || !f_abd) return std::nullopt;
        ++m_stats.swap;
        // (a, b) becomes (c, d): in the face with apex d, b is renamed to c; in the one with apex
        // c, a is renamed to d. Each keeps its corner order, so the two stay oriented alike.
        return Edit{EditKind::swap_edge, {*sa, *sb, *sc, *sd}, {*f_abd, *f_abc}, {}};
    }
}

template <int N>
Edit TrackedRecorder<N>::replacement(
    const std::vector<Cell>& removed,
    const std::vector<Cell>& added,
    std::vector<uint64_t>& fresh)
{
    Edit e{EditKind::replace_faces, {}, {}, {}};
    for (const Cell& t : removed) {
        if (const auto f = lookup(t)) {
            e.faces.push_back(*f);
        } else {
            ++m_stats.unknown;
        }
    }
    std::map<size_t, uint64_t> assigned;
    for (const Cell& t : added) {
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

template <int N>
void TrackedRecorder<N>::commit(Change&& c)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    const std::vector<Cell> before = as_set(c.before), after = as_set(c.after);
    const std::vector<Cell> removed = minus(before, after), added = minus(after, before);
    std::sort(c.written.begin(), c.written.end());
    c.written.erase(std::unique(c.written.begin(), c.written.end()), c.written.end());
    std::vector<uint64_t> fresh;
    // On curves a collapse may remove a feature point that no segment holds: no face changes,
    // but the point goes to the survivor.
    const bool lone_point =
        N == 2 && c.kind == EditKind::collapse_edge && c.v.size() == 2 && live(c.v[0]);
    if (removed.empty() && added.empty() && !lone_point) {
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

template <int N>
void TrackedRecorder<N>::renumber(const std::vector<size_t>& old_to_new)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<uint64_t> next;
    for (size_t v = 0; v < m_complex_of.size(); ++v) {
        const uint64_t s = m_complex_of[v];
        if (s == kNoComplex) continue;
        const size_t n = v < old_to_new.size() ? old_to_new[v] : kNoMesh;
        m_mesh_of[s] = n;
        if (n == kNoMesh) continue;
        if (n >= next.size()) next.resize(n + 1, kNoComplex);
        next[n] = s;
    }
    m_complex_of = std::move(next);
}

template <int N>
size_t TrackedRecorder<N>::resync(const std::vector<Cell>& faces)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    std::vector<Cell> current;
    for (uint64_t f = 0; f < m_replay.faces().size(); ++f) {
        if (!m_replay.face_alive()[f]) continue;
        Cell t;
        for (int k = 0; k < N; ++k) t[k] = m_mesh_of[m_replay.faces()[f][k]];
        current.push_back(t);
    }
    current = as_set(current);
    const std::vector<Cell> target = as_set(faces);
    const std::vector<Cell> removed = minus(current, target), added = minus(target, current);
    if (removed.empty() && added.empty()) return 0;
    std::vector<uint64_t> fresh;
    record(replacement(removed, added, fresh), {}, fresh);
    ++m_stats.replaced;
    return removed.size() + added.size();
}

template <int N>
typename TrackedRecorder<N>::Stats TrackedRecorder<N>::stats() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_stats;
}

template <int N>
std::optional<uint64_t> TrackedRecorder<N>::face_id(const Cell& t) const
{
    return lookup(t);
}

template <int N>
std::optional<uint64_t> TrackedRecorder<N>::vertex_id(const size_t v) const
{
    return live(v);
}

template class TrackedRecorder<2>;
template class TrackedRecorder<3>;

} // namespace wmtk::history
