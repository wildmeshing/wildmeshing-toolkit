#include "Tracking.hpp"

#include <algorithm>

namespace wmtk::components::surface_map {

Tracking::Tracking(const history::SurfaceHistory& h, const Smoothing smoothing)
    : m_map(h, smoothing)
    , m_output_ids(h.output_vertex_ids)
{
    for (uint64_t v = 0; v < m_output_ids.size(); ++v) {
        if (m_output_ids[v] != history::kNoVertex) m_history_vertex[m_output_ids[v]] = v;
    }
}

Tracking Tracking::load(const std::string& path, const Smoothing smoothing)
{
    return Tracking(history::SurfaceHistory::load(path), smoothing);
}

std::vector<std::array<uint64_t, 3>> Tracking::output_triangles() const
{
    std::vector<std::array<uint64_t, 3>> out;
    const auto& faces = m_map.output_faces();
    for (uint64_t f = 0; f < faces.size(); ++f) {
        if (!m_map.output_face_alive()[f]) continue;
        std::array<uint64_t, 3> t = faces[f];
        if (output_by_vertices()) {
            for (uint64_t& v : t) v = v < m_output_ids.size() ? m_output_ids[v] : kNoId;
        }
        out.push_back(t);
    }
    return out;
}

std::vector<OutputPoint> Tracking::to_output(const std::vector<InputPoint>& points) const
{
    std::vector<SurfacePoint> pts;
    pts.reserve(points.size());
    for (const InputPoint& p : points) pts.push_back({p.face, p.bary});
    const std::vector<SurfacePoint> mapped = m_map.to_output(std::move(pts));
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
            out[i].vertices[k] = v < m_output_ids.size() ? m_output_ids[v] : kNoId;
        }
    }
    return out;
}

std::vector<InputPoint> Tracking::to_input(const std::vector<OutputPoint>& points) const
{
    const auto& faces = m_map.output_faces();
    const auto& alive = m_map.output_face_alive();
    // A tet output's point names its face by vertices: find the face of the last state with
    // those corners, and put the coordinates in its corner order.
    std::map<std::array<uint64_t, 3>, uint64_t> face_of;
    if (output_by_vertices()) {
        for (uint64_t f = 0; f < faces.size(); ++f) {
            if (!alive[f]) continue;
            std::array<uint64_t, 3> key = faces[f];
            std::sort(key.begin(), key.end());
            face_of[key] = f;
        }
    }
    std::vector<SurfacePoint> pts(points.size(), SurfacePoint{kNoId, Eigen::Vector3d::Zero()});
    for (size_t i = 0; i < points.size(); ++i) {
        const OutputPoint& p = points[i];
        if (!output_by_vertices()) {
            if (p.face < faces.size() && alive[p.face]) pts[i] = {p.face, p.bary};
            continue;
        }
        std::array<uint64_t, 3> h;
        bool known = true;
        for (int k = 0; k < 3; ++k) {
            const auto it = m_history_vertex.find(p.vertices[k]);
            known = known && it != m_history_vertex.end();
            h[k] = known ? it->second : kNoId;
        }
        if (!known) continue;
        std::array<uint64_t, 3> key = h;
        std::sort(key.begin(), key.end());
        const auto it = face_of.find(key);
        if (it == face_of.end()) continue;
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        for (int c = 0; c < 3; ++c) {
            for (int k = 0; k < 3; ++k) {
                if (faces[it->second][c] == h[k]) b[c] = p.bary[k];
            }
        }
        pts[i] = {it->second, b};
    }
    const std::vector<SurfacePoint> mapped = m_map.to_input(std::move(pts));
    std::vector<InputPoint> out(mapped.size());
    for (size_t i = 0; i < mapped.size(); ++i) {
        if (mapped[i].face < m_map.input_faces().size()) out[i] = {mapped[i].face, mapped[i].bary};
    }
    return out;
}

} // namespace wmtk::components::surface_map
