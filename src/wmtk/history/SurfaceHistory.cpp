#include "SurfaceHistory.hpp"
#include "Replay.hpp"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <optional>

namespace wmtk::history {

namespace {

// Layout (version 6, the first released; the earlier ones were development formats). Every
// count and id is an unsigned LEB128 varint; a position is three little-endian f64.
//
//   "WMTKHIST" u32 version
//   n_vertices, then per vertex: u8 alive, and its position if alive
//   n_faces, then per face: u8 alive, and its three vertex ids if alive
//   n_feature_edges, then per feature edge its two vertex ids
//   n_edits, then per edit: u8 kind, the ids the kind implies --
//       split_edge:    v0 v1 new_v, n_pairs, n_pairs x (f new_f)
//       split_face:    v0 v1 v2 new_v, f new_f1 new_f2
//       collapse_edge: removed kept
//       swap_edge:     v0 v1 a b, fa fb
//       replace_faces: n_removed n_added, the removed faces, then per added face: f c0 c1 c2
//       relabel:       n, then per vertex slot its new id plus one (0: gone); m, then per face
//                      slot the same (m = 0: by rank)
//       embed:         n_faces, then per face: c0 c1 c2, n_sources, the sources; then
//                      n_features and their vertex pairs
//   -- then n_moves and per move: (vid << 1 | midpoint), and the position unless `midpoint`
//   says it is exactly the midpoint of the edit's edge (v0,v1) / (removed,kept), as the
//   vertices were before the edit. Saving and loading both replay the history to know it.
//   metadata length, metadata bytes
//   n_output_vertex_ids, then each plus one (0: none)
constexpr char kMagic[8] = {'W', 'M', 'T', 'K', 'H', 'I', 'S', 'T'};
constexpr uint32_t kVersion = 6;

class Writer
{
public:
    explicit Writer(const std::string& path)
        : m_out(path, std::ios::binary)
        , m_path(path)
    {
        if (!m_out) log_and_throw_error("SurfaceHistory: cannot write {}", path);
    }
    template <typename T>
    void put(const T& x)
    {
        m_out.write(reinterpret_cast<const char*>(&x), sizeof(T));
    }
    void varint(uint64_t n)
    {
        do {
            uint8_t b = n & 0x7F;
            n >>= 7;
            if (n) b |= 0x80;
            put(b);
        } while (n);
    }
    void position(const Eigen::Vector3d& p)
    {
        for (int k = 0; k < 3; ++k) put(p[k]);
    }
    void bytes(const char* p, size_t n) { m_out.write(p, std::streamsize(n)); }
    void finish()
    {
        m_out.flush();
        if (!m_out) log_and_throw_error("SurfaceHistory: writing {} failed", m_path);
    }

private:
    std::ofstream m_out;
    std::string m_path;
};

class Reader
{
public:
    explicit Reader(const std::string& path)
        : m_in(path, std::ios::binary)
        , m_path(path)
    {
        if (!m_in) log_and_throw_error("SurfaceHistory: cannot read {}", path);
        m_in.seekg(0, std::ios::end);
        m_size = uint64_t(m_in.tellg());
        m_in.seekg(0);
    }
    /// The bytes not read yet.
    uint64_t remaining() { return m_size - uint64_t(m_in.tellg()); }
    /// A count of items of at least `min_bytes` each, which the rest of the file must hold: a
    /// damaged count cannot make the reader allocate more than the file could fill.
    uint64_t count(const uint64_t min_bytes = 1)
    {
        const uint64_t n = varint();
        if (n > remaining() / min_bytes) corrupt();
        return n;
    }
    [[noreturn]] void corrupt() const
    {
        log_and_throw_error("SurfaceHistory: {} is corrupt", m_path);
    }
    template <typename T>
    T get()
    {
        T x;
        m_in.read(reinterpret_cast<char*>(&x), sizeof(T));
        if (!m_in) log_and_throw_error("SurfaceHistory: {} is truncated", m_path);
        return x;
    }
    uint64_t varint()
    {
        uint64_t n = 0;
        for (int shift = 0; shift < 64; shift += 7) {
            const uint8_t b = get<uint8_t>();
            n |= uint64_t(b & 0x7F) << shift;
            if (!(b & 0x80)) return n;
        }
        log_and_throw_error("SurfaceHistory: {} holds a malformed integer", m_path);
    }
    Eigen::Vector3d position()
    {
        Eigen::Vector3d p;
        for (int k = 0; k < 3; ++k) p[k] = get<double>();
        return p;
    }
    void bytes(char* p, size_t n)
    {
        m_in.read(p, std::streamsize(n));
        if (!m_in) log_and_throw_error("SurfaceHistory: {} is truncated", m_path);
    }

private:
    std::ifstream m_in;
    std::string m_path;
    uint64_t m_size = 0;
};

/// How many vertex and face ids each kind carries; split_edge's faces come in counted pairs.
size_t vertex_ids(const EditKind k)
{
    switch (k) {
    case EditKind::split_edge: return 3;
    case EditKind::split_face: return 4;
    case EditKind::collapse_edge: return 2;
    case EditKind::swap_edge: return 4;
    default: return 0;
    }
}
size_t face_ids(const EditKind k)
{
    switch (k) {
    case EditKind::split_face: return 3;
    case EditKind::swap_edge: return 2;
    default: return 0;
    }
}

/**
 * @brief Whether edit `e`, as read, can be applied to `r`.
 *
 * Every id it reads names a live vertex or face there. Every id it creates stays within what the
 * rest of the file could still announce: `room` bytes, at least one per id. So a damaged file
 * fails to load instead of making Replay index out of bounds or allocate without end. Moves are
 * checked once read, against `move_limit`.
 */
bool ids_valid(const Edit& e, const Replay& r, const uint64_t room)
{
    const uint64_t nv = r.vertices().size(), nf = r.faces().size();
    const auto live_v = [&](uint64_t v) { return v < nv && r.vertex_alive()[v]; };
    const auto live_f = [&](uint64_t f) { return f < nf && r.face_alive()[f]; };
    const auto new_v = [&](uint64_t v) { return v < nv + room; };
    const auto new_f = [&](uint64_t f) { return f < nf + room; };
    switch (e.kind) {
    case EditKind::split_edge:
        for (size_t i = 0; i + 1 < e.faces.size(); i += 2) {
            if (!live_f(e.faces[i]) || !new_f(e.faces[i + 1])) return false;
        }
        return live_v(e.v[0]) && live_v(e.v[1]) && new_v(e.v[2]);
    case EditKind::split_face:
        return live_v(e.v[0]) && live_v(e.v[1]) && live_v(e.v[2]) && new_v(e.v[3]) &&
               live_f(e.faces[0]) && new_f(e.faces[1]) && new_f(e.faces[2]);
    case EditKind::collapse_edge: return live_v(e.v[0]) && live_v(e.v[1]);
    case EditKind::swap_edge:
        return std::all_of(e.v.begin(), e.v.end(), live_v) &&
               std::all_of(e.faces.begin(), e.faces.end(), live_f);
    case EditKind::move_vertices:
    case EditKind::consolidate: return true;
    case EditKind::replace_faces: {
        // Removed faces that are not live are skipped; added ones may be new.
        const size_t n_added = e.v.size() / 3;
        return std::all_of(e.faces.end() - long(n_added), e.faces.end(), new_f) &&
               std::all_of(e.v.begin(), e.v.end(), new_v);
    }
    case EditKind::relabel: {
        // Ids of the new numbering, which its own lists announce.
        const auto in = [room](const std::vector<uint64_t>& ids) {
            return std::all_of(ids.begin(), ids.end(), [&](uint64_t x) {
                return x == kNoVertex || x < ids.size() + room;
            });
        };
        return in(e.v) && in(e.faces);
    }
    case EditKind::embed: {
        // The new complex's vertices, the corners and the feature edges after the sources.
        const uint64_t limit = e.v.size() + room;
        if (!std::all_of(e.v.begin(), e.v.end(), [&](uint64_t v) { return v < limit; })) {
            return false;
        }
        size_t i = 0;
        for (size_t f = 0; f < e.v.size() / 3; ++f) i += 1 + e.faces[i];
        for (++i; i < e.faces.size(); ++i) {
            if (e.faces[i] >= limit) return false;
        }
        return true;
    }
    default: return false;
    }
}

/// The position a move of this edit is predicted to have: the midpoint of the edit's edge, as
/// the replay holds it before the edit. None for the other kinds.
std::optional<Eigen::Vector3d> predicted(const Edit& e, const Replay& r)
{
    if (e.kind != EditKind::split_edge && e.kind != EditKind::collapse_edge) return std::nullopt;
    return ((r.vertices()[e.v[0]] + r.vertices()[e.v[1]]) / 2.0).eval();
}

} // namespace

Recorder::Recorder(SurfaceHistory start)
    : m_history(std::move(start))
    , m_known(m_history.vertices)
    , m_known_valid(m_history.vertex_alive)
{}

void Recorder::append(Edit&& e)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    // A split's new vertex takes a slot whose last known position belonged to whatever lived
    // there before, so it is always recorded.
    uint64_t fresh = uint64_t(-1);
    if (e.kind == EditKind::split_edge) fresh = e.v[2];
    if (e.kind == EditKind::split_face) fresh = e.v[3];
    auto out = e.moves.begin();
    for (const VertexMove& m : e.moves) {
        if (m.vid >= m_known.size()) {
            m_known.resize(m.vid + 1, Eigen::Vector3d::Zero());
            m_known_valid.resize(m.vid + 1, 0);
        }
        const bool unchanged = m.vid != fresh && m_known_valid[m.vid] && m_known[m.vid] == m.pos;
        m_known[m.vid] = m.pos;
        m_known_valid[m.vid] = 1;
        if (!unchanged) *out++ = m;
    }
    e.moves.erase(out, e.moves.end());
    if (e.kind == EditKind::move_vertices && e.moves.empty()) return;
    m_history.edits.push_back(std::move(e));
}

void Recorder::rebase(std::vector<Eigen::Vector3d> positions)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_known = std::move(positions);
    m_known_valid.assign(m_known.size(), 1);
}

bool append(SurfaceHistory& h, const SurfaceHistory& next)
{
    Replay r(h);
    r.apply_all(h);
    // The live part of each must be the same; dead slots may differ in number.
    const auto& V = r.vertices();
    const auto& F = r.faces();
    for (uint64_t v = 0; v < std::max(V.size(), next.vertices.size()); ++v) {
        const bool a = v < V.size() && r.vertex_alive()[v];
        const bool b = v < next.vertices.size() && next.vertex_alive[v];
        if (a != b || (a && V[v] != next.vertices[v])) {
            logger().error("history append: vertex {} differs where the next history starts", v);
            return false;
        }
    }
    for (uint64_t f = 0; f < std::max(F.size(), next.faces.size()); ++f) {
        const bool a = f < F.size() && r.face_alive()[f];
        const bool b = f < next.faces.size() && next.face_alive[f];
        if (a != b || (a && F[f] != next.faces[f])) {
            logger().error("history append: face {} differs where the next history starts", f);
            return false;
        }
    }
    h.edits.insert(h.edits.end(), next.edits.begin(), next.edits.end());
    return true;
}

void append_embedded(
    SurfaceHistory& h,
    const SurfaceHistory& next,
    const std::vector<std::vector<uint64_t>>& sources)
{
    Edit e{EditKind::embed, {}, {}, {}};
    e.v.reserve(3 * next.faces.size());
    for (size_t f = 0; f < next.faces.size(); ++f) {
        if (!next.face_alive[f]) log_and_throw_error("append_embedded: face {} is not alive", f);
        e.v.insert(e.v.end(), next.faces[f].begin(), next.faces[f].end());
        const auto& src = f < sources.size() ? sources[f] : std::vector<uint64_t>();
        e.faces.push_back(src.size());
        e.faces.insert(e.faces.end(), src.begin(), src.end());
    }
    e.faces.push_back(next.feature_edges.size());
    for (const auto& fe : next.feature_edges) e.faces.insert(e.faces.end(), fe.begin(), fe.end());
    e.moves.reserve(next.vertices.size());
    for (uint64_t v = 0; v < next.vertices.size(); ++v) {
        if (!next.vertex_alive[v])
            log_and_throw_error("append_embedded: vertex {} is not alive", v);
        e.moves.push_back({v, next.vertices[v]});
    }
    h.edits.push_back(std::move(e));
    h.edits.insert(h.edits.end(), next.edits.begin(), next.edits.end());
}

void SurfaceHistory::save(const std::string& path) const
{
    Writer w(path);
    w.bytes(kMagic, sizeof(kMagic));
    w.put(kVersion);

    w.varint(vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
        w.put(vertex_alive[i]);
        if (vertex_alive[i]) w.position(vertices[i]);
    }
    w.varint(faces.size());
    for (size_t i = 0; i < faces.size(); ++i) {
        w.put(face_alive[i]);
        if (!face_alive[i]) continue;
        for (const uint64_t v : faces[i]) w.varint(v);
    }
    w.varint(feature_edges.size());
    for (const auto& e : feature_edges) {
        w.varint(e[0]);
        w.varint(e[1]);
    }

    Replay r(*this);
    w.varint(edits.size());
    for (const Edit& e : edits) {
        w.put(uint8_t(e.kind));
        if (e.kind == EditKind::replace_faces) {
            const size_t n_added = e.v.size() / 3, n_removed = e.faces.size() - n_added;
            w.varint(n_removed);
            w.varint(n_added);
            for (size_t i = 0; i < n_removed; ++i) w.varint(e.faces[i]);
            for (size_t k = 0; k < n_added; ++k) {
                w.varint(e.faces[n_removed + k]);
                for (int c = 0; c < 3; ++c) w.varint(e.v[3 * k + c]);
            }
        }
        if (e.kind == EditKind::relabel) {
            w.varint(e.v.size());
            for (const uint64_t v : e.v) w.varint(v == kNoVertex ? 0 : v + 1);
            w.varint(e.faces.size());
            for (const uint64_t f : e.faces) w.varint(f == kNoVertex ? 0 : f + 1);
        }
        if (e.kind == EditKind::embed) {
            const size_t nf = e.v.size() / 3;
            w.varint(nf);
            size_t i = 0;
            for (size_t f = 0; f < nf; ++f) {
                for (int c = 0; c < 3; ++c) w.varint(e.v[3 * f + c]);
                const uint64_t n = e.faces[i++];
                w.varint(n);
                for (uint64_t k = 0; k < n; ++k) w.varint(e.faces[i++]);
            }
            const uint64_t nfe = i < e.faces.size() ? e.faces[i++] : 0;
            w.varint(nfe);
            for (uint64_t k = 0; k < 2 * nfe; ++k) w.varint(e.faces[i++]);
        }
        const bool generic = e.kind != EditKind::replace_faces && e.kind != EditKind::relabel &&
                             e.kind != EditKind::embed;
        if (generic) {
            for (const uint64_t v : e.v) w.varint(v);
        }
        if (e.kind == EditKind::split_edge) w.varint(e.faces.size() / 2);
        if (generic) {
            for (const uint64_t f : e.faces) w.varint(f);
        }
        const auto mid = predicted(e, r);
        w.varint(e.moves.size());
        for (const VertexMove& m : e.moves) {
            const bool at_mid = mid && m.pos == *mid;
            w.varint(m.vid << 1 | uint64_t(at_mid));
            if (!at_mid) w.position(m.pos);
        }
        r.apply(e);
    }
    w.varint(metadata.size());
    w.bytes(metadata.data(), metadata.size());
    w.varint(output_vertex_ids.size());
    for (const uint64_t v : output_vertex_ids) w.varint(v == kNoVertex ? 0 : v + 1);
    w.finish();
}

SurfaceHistory SurfaceHistory::load(const std::string& path)
{
    Reader r(path);
    char magic[sizeof(kMagic)];
    r.bytes(magic, sizeof(magic));
    if (std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
        log_and_throw_error("SurfaceHistory: {} is not a surface history", path);
    }
    const uint32_t version = r.get<uint32_t>();
    if (version != kVersion) {
        log_and_throw_error(
            "SurfaceHistory: {} has version {}, expected {}",
            path,
            version,
            kVersion);
    }

    SurfaceHistory h;
    const uint64_t nv = r.count();
    h.vertices.assign(nv, Eigen::Vector3d::Zero());
    h.vertex_alive.resize(nv);
    for (uint64_t i = 0; i < nv; ++i) {
        h.vertex_alive[i] = r.get<uint8_t>();
        if (h.vertex_alive[i]) h.vertices[i] = r.position();
    }
    const uint64_t nf = r.count();
    h.faces.assign(nf, {{0, 0, 0}});
    h.face_alive.resize(nf);
    for (uint64_t i = 0; i < nf; ++i) {
        h.face_alive[i] = r.get<uint8_t>();
        if (!h.face_alive[i]) continue;
        for (uint64_t& v : h.faces[i]) {
            v = r.varint();
            if (v >= nv) r.corrupt();
        }
    }
    h.feature_edges.resize(r.count(2));
    for (auto& e : h.feature_edges) {
        e[0] = r.varint();
        e[1] = r.varint();
        if (e[0] >= nv || e[1] >= nv) r.corrupt();
    }

    Replay replay(h);
    h.edits.resize(r.count(2));
    for (Edit& e : h.edits) {
        e.kind = EditKind(r.get<uint8_t>());
        if (e.kind == EditKind::replace_faces) {
            const size_t n_removed = r.count(), n_added = r.count(4);
            e.faces.resize(n_removed + n_added);
            e.v.resize(3 * n_added);
            for (size_t i = 0; i < n_removed; ++i) e.faces[i] = r.varint();
            for (size_t k = 0; k < n_added; ++k) {
                e.faces[n_removed + k] = r.varint();
                for (int c = 0; c < 3; ++c) e.v[3 * k + c] = r.varint();
            }
        } else if (e.kind == EditKind::relabel) {
            e.v.resize(r.count());
            for (uint64_t& v : e.v) {
                const uint64_t n = r.varint();
                v = n == 0 ? kNoVertex : n - 1;
            }
            e.faces.resize(r.count());
            for (uint64_t& f : e.faces) {
                const uint64_t n = r.varint();
                f = n == 0 ? kNoVertex : n - 1;
            }
        } else if (e.kind == EditKind::embed) {
            const size_t nf = r.count(4);
            e.v.resize(3 * nf);
            for (size_t f = 0; f < nf; ++f) {
                for (int c = 0; c < 3; ++c) e.v[3 * f + c] = r.varint();
                const uint64_t n = r.count();
                e.faces.push_back(n);
                for (uint64_t k = 0; k < n; ++k) e.faces.push_back(r.varint());
            }
            const uint64_t nfe = r.count(2);
            e.faces.push_back(nfe);
            for (uint64_t k = 0; k < 2 * nfe; ++k) e.faces.push_back(r.varint());
        } else {
            e.v.resize(vertex_ids(e.kind));
            for (uint64_t& v : e.v) v = r.varint();
            e.faces.resize(
                e.kind == EditKind::split_edge ? 2 * size_t(r.count(2)) : face_ids(e.kind));
            for (uint64_t& f : e.faces) f = r.varint();
        }
        if (!ids_valid(e, replay, r.remaining())) r.corrupt();
        const auto mid = predicted(e, replay);
        e.moves.resize(r.count());
        const uint64_t move_limit = replay.vertices().size() + e.v.size() + r.remaining();
        for (VertexMove& m : e.moves) {
            const uint64_t tagged = r.varint();
            m.vid = tagged >> 1;
            if (m.vid >= move_limit) r.corrupt();
            if (tagged & 1) {
                if (!mid) log_and_throw_error("SurfaceHistory: {} is corrupt", path);
                m.pos = *mid;
            } else {
                m.pos = r.position();
            }
        }
        replay.apply(e);
    }
    h.metadata.resize(r.count());
    r.bytes(h.metadata.data(), h.metadata.size());
    h.output_vertex_ids.resize(r.count());
    for (uint64_t& v : h.output_vertex_ids) {
        const uint64_t n = r.varint();
        v = n == 0 ? kNoVertex : n - 1;
    }
    return h;
}

} // namespace wmtk::history
