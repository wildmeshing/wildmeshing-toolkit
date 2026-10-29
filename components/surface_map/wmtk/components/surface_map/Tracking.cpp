#include "Tracking.hpp"

#include <algorithm>

namespace wmtk::components::surface_map {

Tracking::Tracking(const history::SurfaceHistory& h, const SurfaceMapOptions& options)
    : m_map(h, options)
{
    m_output.ids = h.output_vertex_ids;
    m_input.ids = h.input_vertex_ids;
    for (Naming* n : {&m_output, &m_input}) {
        for (uint64_t v = 0; v < n->ids.size(); ++v) {
            if (n->ids[v] != history::kNoVertex) n->history_vertex[n->ids[v]] = v;
        }
    }
    if (output_by_vertices()) {
        m_output_face_of = faces_by_corners(m_map.output_faces(), m_map.output_face_alive());
    }
    if (input_by_vertices()) {
        m_input_face_of = faces_by_corners(m_map.input_faces(), m_map.input_face_alive());
    }
}

Tracking Tracking::load(const std::string& path, const SurfaceMapOptions& options)
{
    return Tracking(history::SurfaceHistory::load(path), options);
}

std::map<std::array<uint64_t, 3>, uint64_t> Tracking::faces_by_corners(
    const std::vector<std::array<uint64_t, 3>>& faces,
    const std::vector<uint8_t>& alive)
{
    std::map<std::array<uint64_t, 3>, uint64_t> out;
    for (uint64_t f = 0; f < faces.size(); ++f) {
        if (!alive[f]) continue;
        std::array<uint64_t, 3> key = faces[f];
        std::sort(key.begin(), key.end());
        out[key] = f;
    }
    return out;
}

std::vector<std::array<uint64_t, 3>> Tracking::triangles(
    const Naming& naming,
    const std::vector<std::array<uint64_t, 3>>& faces,
    const std::vector<uint8_t>& alive)
{
    std::vector<std::array<uint64_t, 3>> out;
    for (uint64_t f = 0; f < faces.size(); ++f) {
        if (!alive[f]) continue;
        std::array<uint64_t, 3> t = faces[f];
        if (!naming.ids.empty()) {
            for (uint64_t& v : t) v = v < naming.ids.size() ? naming.ids[v] : kNoId;
        }
        out.push_back(t);
    }
    return out;
}

SurfacePoint Tracking::by_vertices(
    const Naming& naming,
    const std::vector<std::array<uint64_t, 3>>& faces,
    const std::map<std::array<uint64_t, 3>, uint64_t>& face_of,
    const std::array<uint64_t, 3>& vertices,
    const Eigen::Vector3d& bary)
{
    // The face of the history with those corners, the coordinates put in its corner order.
    std::array<uint64_t, 3> h;
    for (int k = 0; k < 3; ++k) {
        const auto it = naming.history_vertex.find(vertices[k]);
        if (it == naming.history_vertex.end()) return {kNoId, Eigen::Vector3d::Zero()};
        h[k] = it->second;
    }
    std::array<uint64_t, 3> key = h;
    std::sort(key.begin(), key.end());
    const auto it = face_of.find(key);
    if (it == face_of.end()) return {kNoId, Eigen::Vector3d::Zero()};
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    for (int c = 0; c < 3; ++c) {
        for (int k = 0; k < 3; ++k) {
            if (faces[it->second][c] == h[k]) b[c] = bary[k];
        }
    }
    return {it->second, b};
}

std::vector<std::array<uint64_t, 3>> Tracking::output_triangles() const
{
    return triangles(m_output, m_map.output_faces(), m_map.output_face_alive());
}

std::vector<std::array<uint64_t, 3>> Tracking::input_triangles() const
{
    return triangles(m_input, m_map.input_faces(), m_map.input_face_alive());
}

std::vector<OutputPoint> Tracking::to_output(
    const std::vector<InputPoint>& points,
    MapWarnings* warnings) const
{
    std::vector<SurfacePoint> pts;
    pts.reserve(points.size());
    for (const InputPoint& p : points) {
        pts.push_back(
            input_by_vertices()
                ? by_vertices(m_input, m_map.input_faces(), m_input_face_of, p.vertices, p.bary)
                : SurfacePoint{p.face, p.bary});
    }
    const std::vector<SurfacePoint> mapped = m_map.to_output(std::move(pts), warnings);
    const auto& faces = m_map.output_faces();
    const auto& alive = m_map.output_face_alive();
    std::vector<OutputPoint> out(mapped.size());
    for (size_t i = 0; i < mapped.size(); ++i) {
        const SurfacePoint& p = mapped[i];
        if (p.face >= faces.size() || !alive[p.face]) continue;
        out[i].bary = p.bary;
        if (!output_by_vertices()) {
            out[i].face = p.face;
            continue;
        }
        for (int k = 0; k < 3; ++k) {
            const uint64_t v = faces[p.face][k];
            out[i].vertices[k] = v < m_output.ids.size() ? m_output.ids[v] : kNoId;
        }
    }
    return out;
}

std::vector<InputPoint> Tracking::to_input(
    const std::vector<OutputPoint>& points,
    MapWarnings* warnings) const
{
    const auto& faces = m_map.output_faces();
    const auto& alive = m_map.output_face_alive();
    std::vector<SurfacePoint> pts(points.size(), SurfacePoint{kNoId, Eigen::Vector3d::Zero()});
    for (size_t i = 0; i < points.size(); ++i) {
        const OutputPoint& p = points[i];
        if (output_by_vertices()) {
            pts[i] = by_vertices(m_output, faces, m_output_face_of, p.vertices, p.bary);
        } else if (p.face < faces.size() && alive[p.face]) {
            pts[i] = {p.face, p.bary};
        }
    }
    const std::vector<SurfacePoint> mapped = m_map.to_input(std::move(pts), warnings);
    const auto& in_faces = m_map.input_faces();
    const auto& in_alive = m_map.input_face_alive();
    std::vector<InputPoint> out(mapped.size());
    for (size_t i = 0; i < mapped.size(); ++i) {
        const SurfacePoint& p = mapped[i];
        if (p.face >= in_faces.size() || !in_alive[p.face]) continue;
        out[i].bary = p.bary;
        if (!input_by_vertices()) {
            out[i].face = p.face;
            continue;
        }
        for (int k = 0; k < 3; ++k) {
            const uint64_t v = in_faces[p.face][k];
            out[i].vertices[k] = v < m_input.ids.size() ? m_input.ids[v] : kNoId;
        }
    }
    return out;
}

} // namespace wmtk::components::surface_map
