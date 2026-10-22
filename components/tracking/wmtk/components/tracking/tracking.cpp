#include "tracking.hpp"

#include <algorithm>

namespace wmtk::components::tracking {

Tracking::Tracking(const history::SurfaceHistory& h, const SurfaceMapOptions& options)
    : m_map(h, options)
    , m_output_ids(h.output_vertex_ids)
{
    for (uint64_t v = 0; v < m_output_ids.size(); ++v) {
        if (m_output_ids[v] != history::kNoVertex) m_history_vertex[m_output_ids[v]] = v;
    }
}

Tracking Tracking::load(const std::string& path, const SurfaceMapOptions& options)
{
    return Tracking(history::SurfaceHistory::load(path), options);
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

CurveTracking::CurveTracking(const history::CurveHistory& h, const CurveMapOptions& options)
    : m_map(h, options)
    , m_output_ids(h.output_vertex_ids)
{
    for (uint64_t v = 0; v < m_output_ids.size(); ++v) {
        if (m_output_ids[v] != history::kNoVertex) m_history_vertex[m_output_ids[v]] = v;
    }
}

CurveTracking CurveTracking::load(const std::string& path, const CurveMapOptions& options)
{
    return CurveTracking(history::CurveHistory::load(path), options);
}

std::vector<std::array<uint64_t, 2>> CurveTracking::output_segments() const
{
    std::vector<std::array<uint64_t, 2>> out;
    const auto& segments = m_map.output_segments();
    for (uint64_t f = 0; f < segments.size(); ++f) {
        if (!m_map.output_segment_alive()[f]) continue;
        std::array<uint64_t, 2> s = segments[f];
        if (output_by_vertices()) {
            for (uint64_t& v : s) v = v < m_output_ids.size() ? m_output_ids[v] : kNoId;
        }
        out.push_back(s);
    }
    return out;
}

std::vector<OutputCurvePoint> CurveTracking::to_output(
    const std::vector<InputCurvePoint>& points) const
{
    std::vector<CurvePoint> pts;
    pts.reserve(points.size());
    for (const InputCurvePoint& p : points) pts.push_back({p.segment, p.bary});
    const std::vector<CurvePoint> mapped = m_map.to_output(std::move(pts));
    const auto& segments = m_map.output_segments();
    const auto& alive = m_map.output_segment_alive();
    std::vector<OutputCurvePoint> out(mapped.size());
    for (size_t i = 0; i < mapped.size(); ++i) {
        const CurvePoint& p = mapped[i];
        if (p.segment >= segments.size() || !alive[p.segment]) continue;
        out[i].bary = p.bary;
        if (!output_by_vertices()) {
            out[i].segment = p.segment;
            continue;
        }
        for (int k = 0; k < 2; ++k) {
            const uint64_t v = segments[p.segment][k];
            out[i].vertices[k] = v < m_output_ids.size() ? m_output_ids[v] : kNoId;
        }
    }
    return out;
}

std::vector<InputCurvePoint> CurveTracking::to_input(
    const std::vector<OutputCurvePoint>& points) const
{
    const auto& segments = m_map.output_segments();
    const auto& alive = m_map.output_segment_alive();
    // A point named by vertices: the segment of the last state with those ends, and the
    // coordinates in its order.
    std::map<std::array<uint64_t, 2>, uint64_t> segment_of;
    if (output_by_vertices()) {
        for (uint64_t f = 0; f < segments.size(); ++f) {
            if (!alive[f]) continue;
            const uint64_t a = std::min(segments[f][0], segments[f][1]);
            const uint64_t b = std::max(segments[f][0], segments[f][1]);
            segment_of[{{a, b}}] = f;
        }
    }
    std::vector<CurvePoint> pts(points.size(), CurvePoint{kNoId, Eigen::Vector2d::Zero()});
    for (size_t i = 0; i < points.size(); ++i) {
        const OutputCurvePoint& p = points[i];
        if (!output_by_vertices()) {
            if (p.segment < segments.size() && alive[p.segment]) pts[i] = {p.segment, p.bary};
            continue;
        }
        const auto i0 = m_history_vertex.find(p.vertices[0]);
        const auto i1 = m_history_vertex.find(p.vertices[1]);
        if (i0 == m_history_vertex.end() || i1 == m_history_vertex.end()) continue;
        const uint64_t a = std::min(i0->second, i1->second), b = std::max(i0->second, i1->second);
        const auto it = segment_of.find({{a, b}});
        if (it == segment_of.end()) continue;
        const bool same_order = segments[it->second][0] == i0->second;
        pts[i] = {it->second, same_order ? p.bary : Eigen::Vector2d(p.bary[1], p.bary[0])};
    }
    const std::vector<CurvePoint> mapped = m_map.to_input(std::move(pts));
    std::vector<InputCurvePoint> out(mapped.size());
    for (size_t i = 0; i < mapped.size(); ++i) {
        if (mapped[i].segment < m_map.input_segments().size()) {
            out[i] = {mapped[i].segment, mapped[i].bary};
        }
    }
    return out;
}

std::vector<FeaturePoint> CurveTracking::feature_points() const
{
    std::vector<FeaturePoint> out;
    for (const auto& [in, v] : m_map.feature_points()) {
        uint64_t o = v;
        if (output_by_vertices()) o = v < m_output_ids.size() ? m_output_ids[v] : kNoId;
        out.push_back({in, o == history::kNoVertex ? kNoId : o});
    }
    return out;
}

} // namespace wmtk::components::tracking
