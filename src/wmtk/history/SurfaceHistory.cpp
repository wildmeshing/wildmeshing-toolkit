#include "SurfaceHistory.hpp"

#include <wmtk/utils/Logger.hpp>

#include <cstring>
#include <fstream>

namespace wmtk::history {

namespace {

// Layout, all integers little-endian as the platforms we build on store them:
//   "WMTKHIST" u32 version
//   u64 n_vertices, n_vertices x (f64 x, y, z, u8 alive)
//   u64 n_faces, n_faces x (u64 v0, v1, v2, u8 alive)
//   u64 n_edits, n_edits x (u8 kind, u32 n_v, n_v x u64, u32 n_faces, n_faces x u64,
//                           u32 n_moves, n_moves x (u64 vid, f64 x, y, z))
//   u64 metadata length, metadata bytes
constexpr char kMagic[8] = {'W', 'M', 'T', 'K', 'H', 'I', 'S', 'T'};
constexpr uint32_t kVersion = 1;

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
    }
    template <typename T>
    T get()
    {
        T x;
        m_in.read(reinterpret_cast<char*>(&x), sizeof(T));
        if (!m_in) log_and_throw_error("SurfaceHistory: {} is truncated", m_path);
        return x;
    }
    void bytes(char* p, size_t n)
    {
        m_in.read(p, std::streamsize(n));
        if (!m_in) log_and_throw_error("SurfaceHistory: {} is truncated", m_path);
    }

private:
    std::ifstream m_in;
    std::string m_path;
};

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

void SurfaceHistory::save(const std::string& path) const
{
    Writer w(path);
    w.bytes(kMagic, sizeof(kMagic));
    w.put(kVersion);

    w.put(uint64_t(vertices.size()));
    for (size_t i = 0; i < vertices.size(); ++i) {
        for (int k = 0; k < 3; ++k) w.put(vertices[i][k]);
        w.put(vertex_alive[i]);
    }
    w.put(uint64_t(faces.size()));
    for (size_t i = 0; i < faces.size(); ++i) {
        for (const uint64_t v : faces[i]) w.put(v);
        w.put(face_alive[i]);
    }
    w.put(uint64_t(edits.size()));
    for (const Edit& e : edits) {
        w.put(uint8_t(e.kind));
        w.put(uint32_t(e.v.size()));
        for (const uint64_t v : e.v) w.put(v);
        w.put(uint32_t(e.faces.size()));
        for (const uint64_t f : e.faces) w.put(f);
        w.put(uint32_t(e.moves.size()));
        for (const VertexMove& m : e.moves) {
            w.put(m.vid);
            for (int k = 0; k < 3; ++k) w.put(m.pos[k]);
        }
    }
    w.put(uint64_t(metadata.size()));
    w.bytes(metadata.data(), metadata.size());
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
    const uint64_t nv = r.get<uint64_t>();
    h.vertices.resize(nv);
    h.vertex_alive.resize(nv);
    for (uint64_t i = 0; i < nv; ++i) {
        for (int k = 0; k < 3; ++k) h.vertices[i][k] = r.get<double>();
        h.vertex_alive[i] = r.get<uint8_t>();
    }
    const uint64_t nf = r.get<uint64_t>();
    h.faces.resize(nf);
    h.face_alive.resize(nf);
    for (uint64_t i = 0; i < nf; ++i) {
        for (uint64_t& v : h.faces[i]) v = r.get<uint64_t>();
        h.face_alive[i] = r.get<uint8_t>();
    }
    const uint64_t ne = r.get<uint64_t>();
    h.edits.resize(ne);
    for (Edit& e : h.edits) {
        e.kind = EditKind(r.get<uint8_t>());
        e.v.resize(r.get<uint32_t>());
        for (uint64_t& v : e.v) v = r.get<uint64_t>();
        e.faces.resize(r.get<uint32_t>());
        for (uint64_t& f : e.faces) f = r.get<uint64_t>();
        e.moves.resize(r.get<uint32_t>());
        for (VertexMove& m : e.moves) {
            m.vid = r.get<uint64_t>();
            for (int k = 0; k < 3; ++k) m.pos[k] = r.get<double>();
        }
    }
    h.metadata.resize(r.get<uint64_t>());
    r.bytes(h.metadata.data(), h.metadata.size());
    return h;
}

} // namespace wmtk::history
