#include "transfer.hpp"

#include <wmtk/utils/Logger.hpp>

#include <limits>

namespace wmtk::components::surface_map {

bool matches_input(const SurfaceMap& map, const SurfaceData& input, std::string& why)
{
    const auto& alive = map.input_face_alive();
    size_t n_live = 0;
    for (size_t f = 0; f < alive.size(); ++f) {
        if (!alive[f]) continue;
        if (f != n_live) {
            why = "the map's input faces are not numbered by rank";
            return false;
        }
        ++n_live;
    }
    if (n_live != input.faces.size()) {
        why = "the map starts from " + std::to_string(n_live) + " faces, the file holds " +
              std::to_string(input.faces.size());
        return false;
    }
    for (size_t f = 0; f < n_live; ++f) {
        for (int k = 0; k < 3; ++k) {
            const auto& a = map.input_vertices()[map.input_faces()[f][k]];
            const auto& b = input.vertices[input.faces[f][k]];
            if (a != b) {
                why = "face " + std::to_string(f) + " corner " + std::to_string(k) +
                      " is not where the map has it";
                return false;
            }
        }
    }
    return true;
}

SurfaceData transfer_face_fields(const SurfaceMap& map, const SurfaceData& input)
{
    // The output's live faces, by rank, and their centroids mapped to the input.
    const auto& alive = map.output_face_alive();
    std::vector<uint64_t> out_faces;
    std::vector<SurfacePoint> centroids;
    for (uint64_t f = 0; f < alive.size(); ++f) {
        if (!alive[f]) continue;
        out_faces.push_back(f);
        centroids.push_back({f, Eigen::Vector3d::Constant(1. / 3)});
    }
    const std::vector<SurfacePoint> sources = map.to_input(centroids);

    // The vertices the live faces use, numbered by rank, as a consolidation numbers them.
    SurfaceData out;
    constexpr uint64_t kNone = std::numeric_limits<uint64_t>::max();
    std::vector<uint64_t> rank(map.output_vertices().size(), kNone);
    for (const uint64_t f : out_faces) {
        for (const uint64_t v : map.output_faces()[f]) rank[v] = 0;
    }
    for (uint64_t v = 0; v < rank.size(); ++v) {
        if (rank[v] == kNone) continue;
        rank[v] = out.vertices.size();
        out.vertices.push_back(map.output_vertices()[v]);
    }
    for (const uint64_t f : out_faces) {
        std::array<uint64_t, 3> t = map.output_faces()[f];
        for (uint64_t& v : t) v = rank[v];
        out.faces.push_back(t);
    }
    size_t lost = 0;
    for (const auto& [name, field] : input.face_fields) {
        auto& o = out.face_fields[name];
        o.resize(out_faces.size());
        const std::vector<double> none(field.empty() ? 1 : field.front().size(), 0.);
        for (size_t i = 0; i < out_faces.size(); ++i) {
            const uint64_t g = sources[i].face;
            if (g < field.size()) {
                o[i] = field[g];
            } else {
                o[i] = none;
                ++lost;
            }
        }
    }
    if (lost > 0) logger().warn("transfer: {} face values had no source and were set to 0", lost);
    return out;
}

} // namespace wmtk::components::surface_map
