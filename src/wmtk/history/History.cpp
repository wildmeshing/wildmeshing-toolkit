#include "History.hpp"
#include "BinaryIO.hpp"
#include "Replay.hpp"
#include "VolumeHistory.hpp"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <cstring>
#include <fstream>
#include <optional>

namespace wmtk::history {

namespace {

using detail::Reader;
using detail::Writer;

// Layout (version 7, the first released; the earlier ones were development formats). Every
// count and id is an unsigned LEB128 varint; a position is three little-endian f64. N is the
// number of corners of a face: 3 for a surface, 2 for a curve network.
//
//   "WMTKHIST" u32 version, u8 N
//   n_vertices, then per vertex: u8 alive, and its position if alive
//   n_faces, then per face: u8 alive, and its N vertex ids if alive
//   n_features, then per feature its N - 1 vertex ids
//   n_edits, then per edit: u8 kind, the ids the kind implies --
//       split_edge:    v0 v1 new_v, n_pairs, n_pairs x (f new_f)
//       split_face:    v0 v1 v2 new_v, f new_f1 new_f2 (surfaces only)
//       collapse_edge: removed kept
//       swap_edge:     v0 v1 a b, fa fb (surfaces only)
//       replace_faces: n_removed n_added, the removed faces, then per added face: f and its N
//                      corners
//       relabel:       n, then per vertex slot its new id plus one (0: gone); m, then per face
//                      slot the same (m = 0: by rank)
//       embed:         n_faces, then per face: its N corners, n_sources, the sources; then
//                      n_features and their vertices
//   -- then n_moves and per move: (vid << 1 | midpoint), and the position unless `midpoint`
//   says it is exactly the midpoint of the edit's edge (v0,v1) / (removed,kept), as the
//   vertices were before the edit. Saving and loading both replay the history to know it.
//   metadata length, metadata bytes
//   n_output_vertex_ids, then each plus one (0: none)
//   n_input_vertex_ids, then each plus one (0: none)
constexpr char kMagic[8] = {'W', 'M', 'T', 'K', 'H', 'I', 'S', 'T'};
constexpr uint32_t kVersion = 7;

/// What the messages call a history of N corners a face.
template <int N>
const char* kind_name()
{
    return N == 3 ? "a surface" : "a curve network";
}

/// The magic and the version, then the number of corners of a face, which it returns: refuses
/// anything but a history of this version, of a surface or of a curve network.
int read_header(Reader& r)
{
    char magic[sizeof(kMagic)];
    r.bytes(magic, sizeof(magic));
    if (std::memcmp(magic, kMagic, sizeof(kMagic)) != 0) {
        log_and_throw_error("history: {} is not a history", r.name());
    }
    const uint32_t version = r.get<uint32_t>();
    if (version != kVersion) {
        log_and_throw_error(
            "history: {} has version {}; this build reads version {} only, so the history has "
            "to be recorded again",
            r.name(),
            version,
            kVersion);
    }
    const int n = r.get<uint8_t>();
    if (n != 2 && n != 3) {
        log_and_throw_error(
            "history: {} holds faces of {} corners, neither a surface's 3 nor a curve's 2",
            r.name(),
            n);
    }
    return n;
}

/// What the messages call bytes given in memory.
const std::string kGiven = "the history given";

/// The header of the bytes of a history file: a surface's or curves' (read_header), or the
/// volume history's (is_volume_history).
void check_header(const std::vector<uint8_t>& bytes, const std::string& name)
{
    if (is_volume_history(bytes, name)) return;
    Reader r(bytes, name);
    read_header(r);
}

/// Refuses a volume history where a surface's or curves' is wanted: it is no complex of faces.
void refuse_volume(const std::vector<uint8_t>& bytes, const std::string& name)
{
    if (is_volume_history(bytes, name)) {
        log_and_throw_error(
            "history: {} is a volume history (volume_history_output), not a surface's or "
            "curves'",
            name);
    }
}

/// The history `bytes` hold; `name` for the messages.
template <int N>
History<N> parse(const std::vector<uint8_t>& bytes, const std::string& name);

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
template <int N>
bool ids_valid(const Edit& e, const BasicReplay<N>& r, const uint64_t room)
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
        if (N != 3) return false;
        return live_v(e.v[0]) && live_v(e.v[1]) && live_v(e.v[2]) && new_v(e.v[3]) &&
               live_f(e.faces[0]) && new_f(e.faces[1]) && new_f(e.faces[2]);
    case EditKind::collapse_edge: return live_v(e.v[0]) && live_v(e.v[1]);
    case EditKind::swap_edge:
        if (N != 3) return false;
        return std::all_of(e.v.begin(), e.v.end(), live_v) &&
               std::all_of(e.faces.begin(), e.faces.end(), live_f);
    case EditKind::move_vertices:
    case EditKind::consolidate: return true;
    case EditKind::replace_faces: {
        // Removed faces that are not live are skipped; added ones may be new.
        const size_t n_added = e.v.size() / N;
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
        // The new complex's vertices, the corners and the features after the sources.
        const uint64_t limit = e.v.size() + room;
        if (!std::all_of(e.v.begin(), e.v.end(), [&](uint64_t v) { return v < limit; })) {
            return false;
        }
        size_t i = 0;
        for (size_t f = 0; f < e.v.size() / N; ++f) i += 1 + e.faces[i];
        for (++i; i < e.faces.size(); ++i) {
            if (e.faces[i] >= limit) return false;
        }
        return true;
    }
    default: return false;
    }
}

/// Whether every vertex of the complex an embed brings -- each corner of its faces, each vertex
/// of its features -- is among its moves, which are all the positions it has: a map takes the
/// faces' corners from them.
template <int N>
bool embedded_vertices_placed(const Edit& e)
{
    std::vector<uint8_t> placed;
    for (const VertexMove& m : e.moves) {
        if (m.vid >= placed.size()) placed.resize(m.vid + 1, 0);
        placed[m.vid] = 1;
    }
    const auto at = [&placed](uint64_t v) { return v < placed.size() && placed[v]; };
    if (!std::all_of(e.v.begin(), e.v.end(), at)) return false;
    size_t i = 0;
    for (size_t f = 0; f < e.v.size() / N; ++f) i += 1 + e.faces[i];
    for (++i; i < e.faces.size(); ++i) {
        if (!at(e.faces[i])) return false;
    }
    return true;
}

/// The position a move of this edit is predicted to have: the midpoint of the edit's edge, as
/// the replay holds it before the edit. None for the other kinds.
template <int N>
std::optional<Eigen::Vector3d> predicted(const Edit& e, const BasicReplay<N>& r)
{
    if (e.kind != EditKind::split_edge && e.kind != EditKind::collapse_edge) return std::nullopt;
    return ((r.vertices()[e.v[0]] + r.vertices()[e.v[1]]) / 2.0).eval();
}

} // namespace

template <int N>
BasicRecorder<N>::BasicRecorder(History<N> start)
    : m_history(std::move(start))
    , m_known(m_history.vertices)
    , m_known_valid(m_history.vertex_alive)
{}

template <int N>
void BasicRecorder<N>::append(Edit&& e)
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

template <int N>
void BasicRecorder<N>::rebase(std::vector<Eigen::Vector3d> positions)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_known = std::move(positions);
    m_known_valid.assign(m_known.size(), 1);
}

template <int N>
bool append(History<N>& h, const History<N>& next)
{
    BasicReplay<N> r(h);
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

template <int N>
void append_embedded(
    History<N>& h,
    const History<N>& next,
    const std::vector<std::vector<uint64_t>>& sources)
{
    Edit e{EditKind::embed, {}, {}, {}};
    e.v.reserve(N * next.faces.size());
    for (size_t f = 0; f < next.faces.size(); ++f) {
        if (!next.face_alive[f]) log_and_throw_error("append_embedded: face {} is not alive", f);
        e.v.insert(e.v.end(), next.faces[f].begin(), next.faces[f].end());
        const auto& src = f < sources.size() ? sources[f] : std::vector<uint64_t>();
        e.faces.push_back(src.size());
        e.faces.insert(e.faces.end(), src.begin(), src.end());
    }
    e.faces.push_back(next.features.size());
    for (const auto& fe : next.features) e.faces.insert(e.faces.end(), fe.begin(), fe.end());
    e.moves.reserve(next.vertices.size());
    for (uint64_t v = 0; v < next.vertices.size(); ++v) {
        if (!next.vertex_alive[v])
            log_and_throw_error("append_embedded: vertex {} is not alive", v);
        e.moves.push_back({v, next.vertices[v]});
    }
    h.edits.push_back(std::move(e));
    h.edits.insert(h.edits.end(), next.edits.begin(), next.edits.end());
}

template <int N>
void History<N>::save(const std::string& path) const
{
    write_history(path, to_bytes());
}

template <int N>
History<N> History<N>::load(const std::string& path)
{
    return parse<N>(read_history(path), path);
}

template <int N>
History<N> History<N>::from_bytes(const std::vector<uint8_t>& bytes)
{
    return parse<N>(bytes, kGiven);
}

template <int N>
std::vector<uint8_t> History<N>::to_bytes() const
{
    std::vector<uint8_t> bytes;
    Writer w(bytes);
    w.bytes(kMagic, sizeof(kMagic));
    w.put(kVersion);
    w.put(uint8_t(N));

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
    w.varint(features.size());
    for (const Feature& e : features) {
        for (const uint64_t v : e) w.varint(v);
    }

    BasicReplay<N> r(*this);
    w.varint(edits.size());
    for (const Edit& e : edits) {
        w.put(uint8_t(e.kind));
        if (e.kind == EditKind::replace_faces) {
            const size_t n_added = e.v.size() / N, n_removed = e.faces.size() - n_added;
            w.varint(n_removed);
            w.varint(n_added);
            for (size_t i = 0; i < n_removed; ++i) w.varint(e.faces[i]);
            for (size_t k = 0; k < n_added; ++k) {
                w.varint(e.faces[n_removed + k]);
                for (int c = 0; c < N; ++c) w.varint(e.v[N * k + c]);
            }
        }
        if (e.kind == EditKind::relabel) {
            w.varint(e.v.size());
            for (const uint64_t v : e.v) w.varint(v == kNoVertex ? 0 : v + 1);
            w.varint(e.faces.size());
            for (const uint64_t f : e.faces) w.varint(f == kNoVertex ? 0 : f + 1);
        }
        if (e.kind == EditKind::embed) {
            const size_t nf = e.v.size() / N;
            w.varint(nf);
            size_t i = 0;
            for (size_t f = 0; f < nf; ++f) {
                for (int c = 0; c < N; ++c) w.varint(e.v[N * f + c]);
                const uint64_t n = e.faces[i++];
                w.varint(n);
                for (uint64_t k = 0; k < n; ++k) w.varint(e.faces[i++]);
            }
            const uint64_t nfe = i < e.faces.size() ? e.faces[i++] : 0;
            w.varint(nfe);
            for (uint64_t k = 0; k < (N - 1) * nfe; ++k) w.varint(e.faces[i++]);
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
    w.varint(input_vertex_ids.size());
    for (const uint64_t v : input_vertex_ids) w.varint(v == kNoVertex ? 0 : v + 1);
    return bytes;
}

std::vector<uint8_t> read_history(const std::filesystem::path& path)
{
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    const std::streamoff size = in ? std::streamoff(in.tellg()) : -1;
    if (size < 0) log_and_throw_error("history: cannot read {}", path.string());
    std::vector<uint8_t> bytes(static_cast<size_t>(size));
    in.seekg(0);
    in.read(reinterpret_cast<char*>(bytes.data()), std::streamsize(bytes.size()));
    if (!in) log_and_throw_error("history: reading {} failed", path.string());
    check_header(bytes, path.string());
    return bytes;
}

void write_history(const std::filesystem::path& path, const std::vector<uint8_t>& bytes)
{
    check_header(bytes, kGiven);
    std::ofstream out(path, std::ios::binary);
    if (!out) log_and_throw_error("history: cannot write {}", path.string());
    out.write(reinterpret_cast<const char*>(bytes.data()), std::streamsize(bytes.size()));
    out.flush();
    if (!out) log_and_throw_error("history: writing {} failed", path.string());
}

int face_size(const std::vector<uint8_t>& bytes)
{
    refuse_volume(bytes, kGiven);
    Reader r(bytes, kGiven);
    return read_header(r);
}

int face_size(const std::string& path)
{
    return face_size(read_history(path));
}

namespace {

template <int N>
History<N> parse(const std::vector<uint8_t>& bytes, const std::string& name)
{
    refuse_volume(bytes, name);
    Reader r(bytes, name);
    const int corners = read_header(r);
    if (corners != N) {
        log_and_throw_error(
            "history: {} holds faces of {} corners, {}, not {}'s {}",
            name,
            corners,
            corners == 3 ? "a surface" : "a curve network",
            kind_name<N>(),
            N);
    }

    History<N> h;
    const uint64_t nv = r.count();
    h.vertices.assign(nv, Eigen::Vector3d::Zero());
    h.vertex_alive.resize(nv);
    for (uint64_t i = 0; i < nv; ++i) {
        h.vertex_alive[i] = r.get<uint8_t>();
        if (h.vertex_alive[i]) h.vertices[i] = r.position();
    }
    const uint64_t nf = r.count();
    h.faces.assign(nf, typename History<N>::Face{});
    h.face_alive.resize(nf);
    for (uint64_t i = 0; i < nf; ++i) {
        h.face_alive[i] = r.get<uint8_t>();
        if (!h.face_alive[i]) continue;
        // A live face's corners are live: Replay keeps faces only at the vertices it knows.
        for (uint64_t& v : h.faces[i]) {
            v = r.varint();
            if (v >= nv || !h.vertex_alive[v]) r.corrupt();
        }
    }
    h.features.resize(r.count(N - 1));
    for (typename History<N>::Feature& e : h.features) {
        for (uint64_t& v : e) {
            v = r.varint();
            if (v >= nv) r.corrupt();
        }
    }

    BasicReplay<N> replay(h);
    h.edits.resize(r.count(2));
    for (Edit& e : h.edits) {
        e.kind = EditKind(r.get<uint8_t>());
        if (e.kind == EditKind::replace_faces) {
            const size_t n_removed = r.count(), n_added = r.count(N + 1);
            e.faces.resize(n_removed + n_added);
            e.v.resize(N * n_added);
            for (size_t i = 0; i < n_removed; ++i) e.faces[i] = r.varint();
            for (size_t k = 0; k < n_added; ++k) {
                e.faces[n_removed + k] = r.varint();
                for (int c = 0; c < N; ++c) e.v[N * k + c] = r.varint();
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
            const size_t nf = r.count(N + 1);
            e.v.resize(N * nf);
            for (size_t f = 0; f < nf; ++f) {
                for (int c = 0; c < N; ++c) e.v[N * f + c] = r.varint();
                const uint64_t n = r.count();
                e.faces.push_back(n);
                for (uint64_t k = 0; k < n; ++k) e.faces.push_back(r.varint());
            }
            const uint64_t nfe = r.count(N - 1);
            e.faces.push_back(nfe);
            for (uint64_t k = 0; k < (N - 1) * nfe; ++k) e.faces.push_back(r.varint());
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
                if (!mid) r.corrupt();
                m.pos = *mid;
            } else {
                m.pos = r.position();
            }
        }
        if (e.kind == EditKind::embed && !embedded_vertices_placed<N>(e)) r.corrupt();
        replay.apply(e);
    }
    h.metadata.resize(r.count());
    r.bytes(h.metadata.data(), h.metadata.size());
    for (auto* ids : {&h.output_vertex_ids, &h.input_vertex_ids}) {
        ids->resize(r.count());
        for (uint64_t& v : *ids) {
            const uint64_t n = r.varint();
            v = n == 0 ? kNoVertex : n - 1;
        }
    }
    return h;
}

} // namespace

template struct History<2>;
template struct History<3>;
template class BasicRecorder<2>;
template class BasicRecorder<3>;
template bool append(History<2>&, const History<2>&);
template bool append(History<3>&, const History<3>&);
template void
append_embedded(History<2>&, const History<2>&, const std::vector<std::vector<uint64_t>>&);
template void
append_embedded(History<3>&, const History<3>&, const std::vector<std::vector<uint64_t>>&);

} // namespace wmtk::history
