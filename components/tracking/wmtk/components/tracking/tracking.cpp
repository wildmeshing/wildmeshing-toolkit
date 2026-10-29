#include "tracking.hpp"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>

namespace wmtk::components::tracking {

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
    const std::vector<InputCurvePoint>& points,
    CurveMapWarnings* warnings) const
{
    std::vector<CurvePoint> pts;
    pts.reserve(points.size());
    for (const InputCurvePoint& p : points) pts.push_back({p.segment, p.bary});
    const std::vector<CurvePoint> mapped = m_map.to_output(std::move(pts), warnings);
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
    const std::vector<OutputCurvePoint>& points,
    CurveMapWarnings* warnings) const
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
    const std::vector<CurvePoint> mapped = m_map.to_input(std::move(pts), warnings);
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

nlohmann::json warnings_report(const MapWarnings& w)
{
    return {
        {"total", w.total()},
        {"not_a_disk", w.not_a_disk},
        {"flat_triangle", w.flat_triangle},
        {"inverted", w.inverted},
        {"merged_faces", w.merged_faces},
        {"unlocated", w.unlocated},
        {"replaced", w.replaced}};
}

nlohmann::json warnings_report(const CurveMapWarnings& w)
{
    return {
        {"total", w.total()},
        {"topology", w.topology},
        {"merged_segments", w.merged_segments},
        {"overlapping", w.overlapping},
        {"unlocated", w.unlocated},
        {"replaced", w.replaced}};
}

nlohmann::json stats_report(const VolumeMapStats& s)
{
    return {
        {"built", s.built},
        {"failed", s.failed},
        {"failures", s.failures},
        {"real_pairs", s.real_pairs},
        {"fallback_pairs", s.fallback_pairs},
        {"decompositions", s.decompositions},
        {"surface_not_kept", s.surface_not_kept},
        {"inverted_ops", s.inverted_ops},
        {"region_changed", s.region_changed},
        {"through_failed", s.through_failed},
        {"kept", s.kept},
        {"kept_bytes", s.kept_bytes}};
}

VolumeTracking::VolumeTracking(
    const history::SurfaceHistory& surface,
    history::VolumeHistory volume,
    const MeshData& input_mesh,
    const MeshData& output_mesh,
    const VolumeMapOptions& options)
    : VolumeTracking(surface, std::move(volume), options)
{
    m_in.emplace(std::make_shared<const TetMeshData>(tet_mesh_data(input_mesh)));
    m_out.emplace(std::make_shared<const TetMeshData>(tet_mesh_data(output_mesh)));
}

VolumeTracking::VolumeTracking(
    const history::SurfaceHistory& surface,
    history::VolumeHistory volume,
    const VolumeMapOptions& options)
    : m_map(std::make_shared<const VolumeMap>(surface, std::move(volume), options))
{}

VolumeTracking VolumeTracking::load(
    const std::string& surface_history,
    const std::string& volume_history,
    const std::string& input_mesh,
    const std::string& output_mesh,
    const VolumeMapOptions& options)
{
    return VolumeTracking(
        history::SurfaceHistory::load(surface_history),
        history::VolumeHistory::load(volume_history),
        read_tet_mesh(input_mesh),
        read_tet_mesh(output_mesh),
        options);
}

const TetMeshData& VolumeTracking::input_mesh() const
{
    if (!m_in) log_and_throw_error("VolumeTracking: made without meshes, it has no input mesh");
    return m_in->mesh();
}

const TetMeshData& VolumeTracking::output_mesh() const
{
    if (!m_out) log_and_throw_error("VolumeTracking: made without meshes, it has no output mesh");
    return m_out->mesh();
}

std::vector<VolumePoint> VolumeTracking::map_positions(
    const std::vector<Eigen::Vector3d>& positions,
    const bool forward,
    const std::vector<uint32_t>& regions) const
{
    if (!regions.empty() && regions.size() != positions.size()) {
        log_and_throw_error(
            "VolumeTracking::map_positions: {} regions for {} positions",
            regions.size(),
            positions.size());
    }
    std::vector<VolumePoint> pts(positions.size());
    for (size_t i = 0; i < positions.size(); ++i) {
        pts[i].position = positions[i];
        if (!regions.empty()) pts[i].region = regions[i];
    }
    return forward ? m_map->to_output(std::move(pts)) : m_map->to_input(std::move(pts));
}

std::vector<TetPoint> VolumeTracking::to_output(const std::vector<TetPoint>& points) const
{
    return map_points(points, true);
}

std::vector<TetPoint> VolumeTracking::to_input(const std::vector<TetPoint>& points) const
{
    return map_points(points, false);
}

std::vector<TetPoint> VolumeTracking::map_points(
    const std::vector<TetPoint>& points,
    const bool forward) const
{
    if (!has_meshes()) {
        log_and_throw_error(
            "VolumeTracking: tet points need the meshes; map_positions() maps "
            "positions without them");
    }
    const TetMeshData& from = forward ? input_mesh() : output_mesh();
    const TetMeshData& to = forward ? output_mesh() : input_mesh();
    const TetLocator& locate = forward ? *m_out : *m_in;
    // A point of no tet, or with coordinates that are not numbers, is left without a position,
    // which the map passes by, and comes back with no tet.
    const auto valid = [&](const TetPoint& p) {
        return p.tet < from.tets.size() && p.bary.allFinite();
    };
    std::vector<VolumePoint> pts(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        if (!valid(points[i])) continue;
        const auto& t = from.tets[points[i].tet];
        Eigen::Vector3d x = Eigen::Vector3d::Zero();
        for (int k = 0; k < 4; ++k) x += points[i].bary[k] * from.vertices[t[k]];
        pts[i].position = x;
        // its region, by the labels of the groups the file lists its tet in; else the region
        // of the tet the map finds it in
        const auto r = m_map->region_of_groups(from.group_sets[from.groups[points[i].tet]]);
        if (r) pts[i].region = *r;
    }
    const auto there = forward ? m_map->to_output(pts) : m_map->to_input(pts);
    // The region of each set of groups the other mesh lists tets in.
    std::vector<uint32_t> region_of_set;
    for (const auto& g : to.group_sets) {
        region_of_set.push_back(m_map->region_of_groups(g).value_or(VolumePoint::kAnyRegion));
    }
    std::vector<TetPoint> out(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        if (!valid(points[i])) continue;
        // a tet of the region the point ended in, else of a set of groups with its tet's names,
        // else any holding it
        std::optional<uint32_t> g;
        if (there[i].region != VolumePoint::kAnyRegion) {
            const auto it = std::find(region_of_set.begin(), region_of_set.end(), there[i].region);
            if (it != region_of_set.end()) g = uint32_t(it - region_of_set.begin());
        }
        if (!g) {
            const auto& names = from.group_sets[from.groups[points[i].tet]];
            const auto it = std::find(to.group_sets.begin(), to.group_sets.end(), names);
            if (it != to.group_sets.end()) g = uint32_t(it - to.group_sets.begin());
        }
        const auto [t, b] =
            g ? locate.locate(there[i].position, *g) : locate.locate(there[i].position);
        if (t < 0) continue;
        out[i] = {uint64_t(t), b};
    }
    return out;
}

} // namespace wmtk::components::tracking
