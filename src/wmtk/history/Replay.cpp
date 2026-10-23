#include "Replay.hpp"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <map>

namespace wmtk::history {

template <int N>
BasicReplay<N>::BasicReplay(const History<N>& h)
    : m_vertices(h.vertices)
    , m_vertex_alive(h.vertex_alive)
    , m_faces(h.faces)
    , m_face_alive(h.face_alive)
    , m_vertex_faces(h.vertices.size())
{
    for (uint64_t f = 0; f < m_faces.size(); ++f) {
        if (!m_face_alive[f]) continue;
        for (const uint64_t v : m_faces[f]) m_vertex_faces[v].push_back(f);
    }
    m_feature_nbrs.resize(m_vertices.size());
    for (const Feature& e : h.features) add_feature(e);
}

template <int N>
void BasicReplay<N>::add_feature(const Feature& f)
{
    if constexpr (N == 3) {
        const uint64_t a = f[0], b = f[1];
        grow_vertices(std::max(a, b));
        for (const auto& [x, y] : {std::pair{a, b}, std::pair{b, a}}) {
            auto& n = m_feature_nbrs[x];
            if (std::find(n.begin(), n.end(), y) == n.end()) n.push_back(y);
        }
    } else {
        grow_vertices(f[0]);
        m_feature_nbrs[f[0]] = {f[0]};
    }
}

template <int N>
bool BasicReplay<N>::remove_feature(const Feature& f)
{
    const uint64_t a = f[0], b = f[N - 2];
    if (a >= m_feature_nbrs.size() || b >= m_feature_nbrs.size()) return false;
    bool found = false;
    for (const auto& [x, y] : {std::pair{a, b}, std::pair{b, a}}) {
        auto& n = m_feature_nbrs[x];
        const auto it = std::find(n.begin(), n.end(), y);
        if (it == n.end()) continue;
        n.erase(it);
        found = true;
    }
    return found;
}

template <int N>
void BasicReplay<N>::rename_features(const uint64_t from, const uint64_t to)
{
    if (from >= m_feature_nbrs.size()) return;
    const std::vector<uint64_t> nbrs = m_feature_nbrs[from];
    if constexpr (N == 3) {
        for (const uint64_t x : nbrs) {
            remove_feature({{from, x}});
            if (x != to) add_feature({{to, x}});
        }
    } else {
        if (nbrs.empty()) return;
        m_feature_nbrs[from].clear();
        add_feature({{to}});
    }
}

template <int N>
void BasicReplay<N>::drop_features(const uint64_t v)
{
    if (v >= m_feature_nbrs.size()) return;
    const std::vector<uint64_t> nbrs = m_feature_nbrs[v];
    for (const uint64_t x : nbrs) {
        if constexpr (N == 3) {
            remove_feature({{v, x}});
        } else {
            remove_feature({{v}});
        }
    }
}

template <int N>
bool BasicReplay<N>::is_feature(Feature f) const
{
    std::sort(f.begin(), f.end());
    const uint64_t a = f[0], b = f[N - 2];
    if (a >= m_feature_nbrs.size()) return false;
    const auto& n = m_feature_nbrs[a];
    return std::find(n.begin(), n.end(), b) != n.end();
}

template <int N>
std::set<typename BasicReplay<N>::Feature> BasicReplay<N>::features() const
{
    std::set<Feature> out;
    for (uint64_t a = 0; a < m_feature_nbrs.size(); ++a) {
        for (const uint64_t b : m_feature_nbrs[a]) {
            if constexpr (N == 3) {
                if (a < b) out.insert({{a, b}});
            } else {
                out.insert({{a}});
            }
        }
    }
    return out;
}

template <int N>
void BasicReplay<N>::apply_all(const History<N>& h)
{
    for (const Edit& e : h.edits) apply(e);
}

template <int N>
void BasicReplay<N>::apply(const Edit& e)
{
    switch (e.kind) {
    case EditKind::split_edge: split_edge(e); break;
    case EditKind::split_face: split_face(e); break;
    case EditKind::collapse_edge: collapse_edge(e); break;
    case EditKind::swap_edge: swap_edge(e); break;
    case EditKind::move_vertices: break;
    case EditKind::consolidate: consolidate(); break;
    case EditKind::replace_faces: replace_faces(e); break;
    case EditKind::relabel: relabel(e); break;
    case EditKind::embed: embed(e); break;
    default: log_and_throw_error("Replay: unknown edit kind {}", int(e.kind));
    }
    for (const VertexMove& m : e.moves) {
        grow_vertices(m.vid);
        m_vertices[m.vid] = m.pos;
    }
}

template <int N>
void BasicReplay<N>::grow_vertices(const uint64_t vid)
{
    if (vid < m_vertices.size()) return;
    if (vid == kNoVertex) log_and_throw_error("Replay: an edit names no vertex as one");
    m_vertices.resize(vid + 1, Eigen::Vector3d::Zero());
    m_vertex_alive.resize(vid + 1, 0);
    m_vertex_faces.resize(vid + 1);
    m_feature_nbrs.resize(vid + 1);
}

template <int N>
void BasicReplay<N>::grow_faces(const uint64_t fid)
{
    if (fid < m_faces.size()) return;
    m_faces.resize(fid + 1, Face{});
    m_face_alive.resize(fid + 1, 0);
}

template <int N>
void BasicReplay<N>::link(const uint64_t v, const uint64_t f)
{
    auto& fs = m_vertex_faces[v];
    const auto it = std::lower_bound(fs.begin(), fs.end(), f);
    if (it == fs.end() || *it != f) fs.insert(it, f);
}

template <int N>
void BasicReplay<N>::unlink(const uint64_t v, const uint64_t f)
{
    auto& fs = m_vertex_faces[v];
    const auto it = std::lower_bound(fs.begin(), fs.end(), f);
    if (it != fs.end() && *it == f) fs.erase(it);
}

template <int N>
void BasicReplay<N>::rename_in_face(const uint64_t f, const uint64_t from, const uint64_t to)
{
    for (uint64_t& v : m_faces[f]) {
        if (v == from) v = to;
    }
}

template <int N>
void BasicReplay<N>::retire_bare(const std::vector<uint64_t>& vs)
{
    for (const uint64_t v : vs) {
        if (!m_vertex_faces[v].empty()) continue;
        // A feature point holds its vertex without any segment; a feature edge, which lies on
        // faces, goes with its vertex -- a consolidation would have nowhere to put it.
        if (N == 2 && !m_feature_nbrs[v].empty()) continue;
        m_vertex_alive[v] = 0;
        drop_features(v);
    }
}

template <int N>
void BasicReplay<N>::split_edge(const Edit& e)
{
    const uint64_t v0 = e.v[0], v1 = e.v[1], vn = e.v[2];
    grow_vertices(vn);
    m_vertex_alive[vn] = 1;
    if constexpr (N == 3) {
        if (remove_feature({{v0, v1}})) {
            add_feature({{v0, vn}});
            add_feature({{vn, v1}});
        }
    }
    for (size_t i = 0; i + 1 < e.faces.size(); i += 2) {
        const uint64_t f = e.faces[i], fn = e.faces[i + 1];
        grow_faces(fn);
        m_faces[fn] = m_faces[f];
        rename_in_face(fn, v0, vn);
        rename_in_face(f, v1, vn);
        m_face_alive[fn] = 1;
        unlink(v1, f);
        for (const uint64_t v : m_faces[fn]) link(v, fn);
        link(vn, f);
    }
}

template <int N>
void BasicReplay<N>::split_face(const Edit& e)
{
    if constexpr (N != 3) {
        log_and_throw_error("Replay: a face split on curves");
    } else {
        const uint64_t vn = e.v[3];
        const uint64_t f = e.faces[0];
        grow_vertices(vn);
        m_vertex_alive[vn] = 1;
        const Face original = m_faces[f];
        for (size_t k = 0; k < 3; ++k) {
            const uint64_t fk = e.faces[k];
            grow_faces(fk);
            m_faces[fk] = original;
            rename_in_face(fk, e.v[k], vn);
            m_face_alive[fk] = 1;
        }
        unlink(e.v[0], f);
        for (size_t k = 1; k < 3; ++k) {
            for (const uint64_t v : m_faces[e.faces[k]]) link(v, e.faces[k]);
        }
        link(vn, f);
    }
}

template <int N>
void BasicReplay<N>::collapse_edge(const Edit& e)
{
    const uint64_t removed = e.v[0], kept = e.v[1];
    // The removed endpoint's features go to the survivor; the collapsed edge goes.
    rename_features(removed, kept);
    std::vector<uint64_t> touched{kept};
    const std::vector<uint64_t> fs = m_vertex_faces[removed];
    for (const uint64_t f : fs) {
        const bool has_kept =
            std::find(m_faces[f].begin(), m_faces[f].end(), kept) != m_faces[f].end();
        if (has_kept) {
            m_face_alive[f] = 0;
            for (const uint64_t v : m_faces[f]) {
                unlink(v, f);
                touched.push_back(v);
            }
        } else {
            rename_in_face(f, removed, kept);
            link(kept, f);
        }
    }
    m_vertex_faces[removed].clear();
    m_vertex_alive[removed] = 0;

    // Duplicates the collapse made at `kept` merge into the smallest fid.
    std::map<Face, uint64_t> first_with_key;
    const std::vector<uint64_t> at_kept = m_vertex_faces[kept];
    for (const uint64_t f : at_kept) {
        Face key = m_faces[f];
        std::sort(key.begin(), key.end());
        if (first_with_key.try_emplace(key, f).second) continue;
        m_face_alive[f] = 0;
        for (const uint64_t v : m_faces[f]) {
            unlink(v, f);
            touched.push_back(v);
        }
    }

    touched.erase(std::remove(touched.begin(), touched.end(), removed), touched.end());
    retire_bare(touched);
}

template <int N>
void BasicReplay<N>::swap_edge(const Edit& e)
{
    if constexpr (N != 3) {
        log_and_throw_error("Replay: an edge swap on curves");
    } else {
        const uint64_t v0 = e.v[0], v1 = e.v[1], a = e.v[2], b = e.v[3];
        remove_feature({{v0, v1}}); // never a feature edge: the meshes refuse to swap one
        const uint64_t fa = e.faces[0], fb = e.faces[1];
        rename_in_face(fa, v1, a);
        rename_in_face(fb, v0, b);
        unlink(v1, fa);
        unlink(v0, fb);
        link(a, fa);
        link(b, fb);
    }
}

template <int N>
void BasicReplay<N>::replace_faces(const Edit& e)
{
    const size_t n_added = e.v.size() / N;
    const size_t n_removed = e.faces.size() - n_added;
    std::vector<uint64_t> touched;
    for (size_t i = 0; i < n_removed; ++i) {
        const uint64_t f = e.faces[i];
        if (f >= m_faces.size() || !m_face_alive[f]) continue;
        m_face_alive[f] = 0;
        for (const uint64_t v : m_faces[f]) {
            unlink(v, f);
            touched.push_back(v);
        }
    }
    for (size_t k = 0; k < n_added; ++k) {
        const uint64_t f = e.faces[n_removed + k];
        grow_faces(f);
        for (int c = 0; c < N; ++c) {
            const uint64_t v = e.v[N * k + c];
            grow_vertices(v);
            m_faces[f][c] = v;
            m_vertex_alive[v] = 1;
            link(v, f);
        }
        m_face_alive[f] = 1;
    }
    retire_bare(touched);
}

template <int N>
void BasicReplay<N>::relabel(const Edit& e)
{
    uint64_t nv = 0;
    for (const uint64_t n : e.v) {
        if (n != kNoVertex) nv = std::max(nv, n + 1);
    }
    const auto renamed = [&e](const uint64_t v) { return v < e.v.size() ? e.v[v] : kNoVertex; };
    std::vector<Face> faces;
    std::vector<uint8_t> placed;
    for (uint64_t f = 0; f < m_faces.size(); ++f) {
        if (!m_face_alive[f]) continue;
        Face t = m_faces[f];
        for (uint64_t& v : t) v = renamed(v);
        Face s = t;
        std::sort(s.begin(), s.end());
        if (s[N - 1] == kNoVertex || std::adjacent_find(s.begin(), s.end()) != s.end()) continue;
        if (e.faces.empty()) {
            faces.push_back(t);
            continue;
        }
        const uint64_t g = f < e.faces.size() ? e.faces[f] : kNoVertex;
        if (g == kNoVertex) continue;
        if (g >= faces.size()) {
            faces.resize(g + 1, Face{});
            placed.resize(g + 1, 0);
        }
        if (placed[g]) continue; // a duplicate: the face keeps the first's corners
        faces[g] = t;
        placed[g] = 1;
    }
    if (!e.faces.empty() && std::count(placed.begin(), placed.end(), 0) > 0) {
        log_and_throw_error("Replay: a relabel leaves face ids unused");
    }
    std::vector<std::vector<uint64_t>> nbrs(nv);
    for (uint64_t a = 0; a < m_feature_nbrs.size(); ++a) {
        for (const uint64_t b : m_feature_nbrs[a]) {
            const uint64_t na = renamed(a), nb = renamed(b);
            if (na == kNoVertex || nb == kNoVertex || (N == 3 && na == nb)) continue;
            if (std::find(nbrs[na].begin(), nbrs[na].end(), nb) == nbrs[na].end()) {
                nbrs[na].push_back(nb);
            }
        }
    }
    m_vertices.assign(nv, Eigen::Vector3d::Zero());
    m_vertex_alive.assign(nv, 0);
    m_faces = std::move(faces);
    m_face_alive.assign(m_faces.size(), 1);
    m_vertex_faces.assign(nv, {});
    for (uint64_t f = 0; f < m_faces.size(); ++f) {
        for (const uint64_t v : m_faces[f]) {
            m_vertex_faces[v].push_back(f);
            m_vertex_alive[v] = 1;
        }
    }
    m_feature_nbrs = std::move(nbrs);
    if (N == 2) {
        for (uint64_t v = 0; v < nv; ++v) {
            if (!m_feature_nbrs[v].empty()) m_vertex_alive[v] = 1;
        }
    }
}

template <int N>
void BasicReplay<N>::embed(const Edit& e)
{
    const size_t nf = e.v.size() / N;
    uint64_t nv = 0;
    for (const uint64_t v : e.v) nv = std::max(nv, v + 1);
    for (const VertexMove& m : e.moves) nv = std::max(nv, m.vid + 1);
    m_vertices.assign(nv, Eigen::Vector3d::Zero());
    m_vertex_alive.assign(nv, 0);
    m_faces.assign(nf, Face{});
    m_face_alive.assign(nf, 1);
    m_vertex_faces.assign(nv, {});
    m_feature_nbrs.assign(nv, {});
    for (uint64_t f = 0; f < nf; ++f) {
        for (int k = 0; k < N; ++k) {
            const uint64_t v = e.v[N * f + k];
            m_faces[f][k] = v;
            m_vertex_faces[v].push_back(f);
            m_vertex_alive[v] = 1;
        }
    }
    size_t i = 0;
    for (uint64_t f = 0; f < nf; ++f) i += 1 + e.faces[i]; // the sources
    if (i < e.faces.size()) {
        const uint64_t n = e.faces[i++];
        for (uint64_t k = 0; k < n; ++k, i += N - 1) {
            Feature fe;
            for (int c = 0; c < N - 1; ++c) fe[c] = e.faces[i + c];
            add_feature(fe);
            if (N == 2) m_vertex_alive[fe[0]] = 1;
        }
    }
}

template <int N>
void BasicReplay<N>::consolidate()
{
    std::vector<uint64_t> map_v(m_vertices.size(), uint64_t(-1));
    uint64_t nv = 0;
    for (uint64_t v = 0; v < m_vertices.size(); ++v) {
        if (m_vertex_alive[v]) map_v[v] = nv++;
    }
    std::vector<Eigen::Vector3d> vertices(nv);
    for (uint64_t v = 0; v < m_vertices.size(); ++v) {
        if (m_vertex_alive[v]) vertices[map_v[v]] = m_vertices[v];
    }
    std::vector<Face> faces;
    for (uint64_t f = 0; f < m_faces.size(); ++f) {
        if (!m_face_alive[f]) continue;
        Face t = m_faces[f];
        for (uint64_t& v : t) v = map_v[v];
        faces.push_back(t);
    }
    m_vertices = std::move(vertices);
    m_vertex_alive.assign(nv, 1);
    m_faces = std::move(faces);
    m_face_alive.assign(m_faces.size(), 1);
    m_vertex_faces.assign(nv, {});
    for (uint64_t f = 0; f < m_faces.size(); ++f) {
        for (const uint64_t v : m_faces[f]) m_vertex_faces[v].push_back(f);
    }
    std::vector<std::vector<uint64_t>> nbrs(nv);
    for (uint64_t a = 0; a < m_feature_nbrs.size(); ++a) {
        if (map_v[a] == uint64_t(-1)) continue;
        for (const uint64_t b : m_feature_nbrs[a]) {
            if (map_v[b] != uint64_t(-1)) nbrs[map_v[a]].push_back(map_v[b]);
        }
    }
    m_feature_nbrs = std::move(nbrs);
}

template class BasicReplay<2>;
template class BasicReplay<3>;

} // namespace wmtk::history
