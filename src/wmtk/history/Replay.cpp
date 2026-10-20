#include "Replay.hpp"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <map>

namespace wmtk::history {

Replay::Replay(const SurfaceHistory& h)
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
}

void Replay::apply_all(const SurfaceHistory& h)
{
    for (const Edit& e : h.edits) apply(e);
}

void Replay::apply(const Edit& e)
{
    switch (e.kind) {
    case EditKind::split_edge: split_edge(e); break;
    case EditKind::split_face: split_face(e); break;
    case EditKind::collapse_edge: collapse_edge(e); break;
    case EditKind::swap_edge: swap_edge(e); break;
    case EditKind::move_vertices: break;
    case EditKind::consolidate: consolidate(); break;
    default: log_and_throw_error("Replay: unknown edit kind {}", int(e.kind));
    }
    for (const VertexMove& m : e.moves) {
        grow_vertices(m.vid);
        m_vertices[m.vid] = m.pos;
    }
}

void Replay::grow_vertices(const uint64_t vid)
{
    if (vid < m_vertices.size()) return;
    m_vertices.resize(vid + 1, Eigen::Vector3d::Zero());
    m_vertex_alive.resize(vid + 1, 0);
    m_vertex_faces.resize(vid + 1);
}

void Replay::grow_faces(const uint64_t fid)
{
    if (fid < m_faces.size()) return;
    m_faces.resize(fid + 1, {{0, 0, 0}});
    m_face_alive.resize(fid + 1, 0);
}

void Replay::link(const uint64_t v, const uint64_t f)
{
    auto& fs = m_vertex_faces[v];
    const auto it = std::lower_bound(fs.begin(), fs.end(), f);
    if (it == fs.end() || *it != f) fs.insert(it, f);
}

void Replay::unlink(const uint64_t v, const uint64_t f)
{
    auto& fs = m_vertex_faces[v];
    const auto it = std::lower_bound(fs.begin(), fs.end(), f);
    if (it != fs.end() && *it == f) fs.erase(it);
}

void Replay::rename_in_face(const uint64_t f, const uint64_t from, const uint64_t to)
{
    for (uint64_t& v : m_faces[f]) {
        if (v == from) v = to;
    }
}

void Replay::split_edge(const Edit& e)
{
    const uint64_t v0 = e.v[0], v1 = e.v[1], vn = e.v[2];
    grow_vertices(vn);
    m_vertex_alive[vn] = 1;
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

void Replay::split_face(const Edit& e)
{
    const uint64_t vn = e.v[3];
    const uint64_t f = e.faces[0];
    grow_vertices(vn);
    m_vertex_alive[vn] = 1;
    const std::array<uint64_t, 3> original = m_faces[f];
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

void Replay::collapse_edge(const Edit& e)
{
    const uint64_t removed = e.v[0], kept = e.v[1];
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
    std::map<std::array<uint64_t, 3>, uint64_t> first_with_key;
    const std::vector<uint64_t> at_kept = m_vertex_faces[kept];
    for (const uint64_t f : at_kept) {
        std::array<uint64_t, 3> key = m_faces[f];
        std::sort(key.begin(), key.end());
        if (first_with_key.try_emplace(key, f).second) continue;
        m_face_alive[f] = 0;
        for (const uint64_t v : m_faces[f]) {
            unlink(v, f);
            touched.push_back(v);
        }
    }

    for (const uint64_t v : touched) {
        if (v != removed && m_vertex_faces[v].empty()) m_vertex_alive[v] = 0;
    }
}

void Replay::swap_edge(const Edit& e)
{
    const uint64_t v0 = e.v[0], v1 = e.v[1], a = e.v[2], b = e.v[3];
    const uint64_t fa = e.faces[0], fb = e.faces[1];
    rename_in_face(fa, v1, a);
    rename_in_face(fb, v0, b);
    unlink(v1, fa);
    unlink(v0, fb);
    link(a, fa);
    link(b, fb);
}

void Replay::consolidate()
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
    std::vector<std::array<uint64_t, 3>> faces;
    for (uint64_t f = 0; f < m_faces.size(); ++f) {
        if (!m_face_alive[f]) continue;
        std::array<uint64_t, 3> t = m_faces[f];
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
}

} // namespace wmtk::history
