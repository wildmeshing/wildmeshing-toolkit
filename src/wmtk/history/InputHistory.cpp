#include "InputHistory.hpp"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>

namespace wmtk::history {

SurfaceHistory from_input(
    const io::ReadProvenance& read,
    const std::vector<Eigen::Vector3d>& welded,
    const SurfaceHistory& recorded)
{
    SurfaceHistory chain;
    for (Eigen::Index v = 0; v < read.V.rows(); ++v) {
        chain.vertices.push_back(read.V.row(v).transpose());
    }
    chain.vertex_alive.assign(chain.vertices.size(), 1);
    for (Eigen::Index f = 0; f < read.F.rows(); ++f) {
        chain.faces.push_back(
            {{uint64_t(read.F(f, 0)), uint64_t(read.F(f, 1)), uint64_t(read.F(f, 2))}});
    }
    chain.face_alive.assign(chain.faces.size(), 1);

    // The feature curves on vertices of the file, which the weld takes onto the mesh's.
    std::vector<uint64_t> raw_of(welded.size(), kNoVertex);
    for (size_t r = 0; r < read.vertex_map.size(); ++r) {
        const int w = read.vertex_map[r];
        if (w >= 0 && size_t(w) < raw_of.size() && raw_of[w] == kNoVertex) raw_of[w] = r;
    }
    for (const auto& e : recorded.features) {
        std::array<uint64_t, 2> raw = {{raw_of[e[0]], raw_of[e[1]]}};
        if (raw[0] > raw[1]) std::swap(raw[0], raw[1]);
        chain.features.push_back(raw);
    }

    Edit weld{EditKind::relabel, {}, {}, {}};
    for (const int w : read.vertex_map) weld.v.push_back(w < 0 ? kNoVertex : uint64_t(w));
    for (uint64_t v = 0; v < welded.size(); ++v) weld.moves.push_back({v, welded[v]});
    chain.edits.push_back(std::move(weld));
    if (!append(chain, recorded)) {
        log_and_throw_error("history: the recorded history does not start at the input as read");
    }
    chain.metadata = recorded.metadata;
    chain.output_vertex_ids = recorded.output_vertex_ids;
    return chain;
}

} // namespace wmtk::history
