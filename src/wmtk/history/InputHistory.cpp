#include "InputHistory.hpp"
#include "Replay.hpp"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <map>

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
    for (Eigen::Index f = 0; f < read.cells.rows(); ++f) {
        chain.faces.push_back(
            {{uint64_t(read.cells(f, 0)), uint64_t(read.cells(f, 1)), uint64_t(read.cells(f, 2))}});
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

namespace {
Eigen::Vector3d point_of(const MatrixXd& V, const Eigen::Index v)
{
    return Eigen::Vector3d(V(v, 0), V(v, 1), 0);
}
} // namespace

CurveHistory curves_from_input(
    const io::ReadProvenance& read,
    const MatrixXd& V_in,
    const MatrixXi& E_in,
    const std::vector<Edit>& collapses,
    const MatrixXd& V_out,
    const MatrixXi& E_out,
    const std::vector<int>& surviving_vertices)
{
    CurveHistory h;
    for (Eigen::Index v = 0; v < read.V.rows(); ++v) h.vertices.push_back(point_of(read.V, v));
    h.vertex_alive.assign(h.vertices.size(), 1);
    for (Eigen::Index e = 0; e < read.cells.rows(); ++e) {
        h.faces.push_back({{uint64_t(read.cells(e, 0)), uint64_t(read.cells(e, 1))}});
    }
    h.face_alive.assign(h.faces.size(), 1);

    // The feature points, on the welded network, each on the first vertex of the files that
    // became it.
    std::vector<int> valence(V_in.rows(), 0);
    for (Eigen::Index e = 0; e < E_in.rows(); ++e) {
        ++valence[E_in(e, 0)];
        ++valence[E_in(e, 1)];
    }
    std::vector<uint64_t> raw_of(V_in.rows(), kNoVertex);
    for (size_t r = 0; r < read.vertex_map.size(); ++r) {
        const int w = read.vertex_map[r];
        if (w >= 0 && w < int(raw_of.size()) && raw_of[w] == kNoVertex) raw_of[w] = r;
    }
    for (Eigen::Index w = 0; w < V_in.rows(); ++w) {
        if (valence[w] != 2 && raw_of[w] != kNoVertex) h.features.push_back({{raw_of[w]}});
    }
    std::sort(h.features.begin(), h.features.end());

    // The weld: each vertex and segment renamed to the one it became; a segment left with one
    // vertex goes, and a duplicate becomes the segment it duplicates.
    Edit weld{EditKind::relabel, {}, {}, {}};
    for (const int w : read.vertex_map) weld.v.push_back(w < 0 ? kNoVertex : uint64_t(w));
    for (const int e : read.cell_map) weld.faces.push_back(e < 0 ? kNoVertex : uint64_t(e));
    for (Eigen::Index v = 0; v < V_in.rows(); ++v)
        weld.moves.push_back({uint64_t(v), point_of(V_in, v)});
    h.edits.push_back(std::move(weld));
    if (collapses.empty()) return h;

    // The simplification, then its compaction: rows by rank, each segment onto the row with its
    // vertices. A collapse that leaves two copies of a segment keeps the smaller id here
    // (EditKind::collapse_edge) and the one it did not move there, so rows are found by vertices.
    h.edits.insert(h.edits.end(), collapses.begin(), collapses.end());
    CurveReplay r(h);
    r.apply_all(h);
    Edit compact{EditKind::relabel, {}, {}, {}};
    compact.v.assign(r.vertices().size(), kNoVertex);
    for (size_t k = 0; k < surviving_vertices.size(); ++k) {
        const int v = surviving_vertices[k];
        if (v < 0 || size_t(v) >= compact.v.size() || !r.vertex_alive()[v]) {
            log_and_throw_error("curve history: the simplification kept a vertex it removed");
        }
        compact.v[v] = k;
    }
    std::map<std::array<uint64_t, 2>, uint64_t> row_of;
    for (Eigen::Index e = 0; e < E_out.rows(); ++e) {
        const uint64_t a = uint64_t(std::min(E_out(e, 0), E_out(e, 1)));
        const uint64_t b = uint64_t(std::max(E_out(e, 0), E_out(e, 1)));
        row_of[{{a, b}}] = uint64_t(e);
    }
    std::vector<uint8_t> placed(E_out.rows(), 0);
    compact.faces.assign(r.faces().size(), kNoVertex);
    for (uint64_t f = 0; f < r.faces().size(); ++f) {
        if (!r.face_alive()[f]) continue;
        const uint64_t a = compact.v[r.faces()[f][0]], b = compact.v[r.faces()[f][1]];
        const auto it = row_of.find({{std::min(a, b), std::max(a, b)}});
        if (a == kNoVertex || b == kNoVertex || it == row_of.end() || placed[it->second]) {
            log_and_throw_error("curve history: the collapses do not leave the simplified curves");
        }
        compact.faces[f] = it->second;
        placed[it->second] = 1;
    }
    if (std::count(placed.begin(), placed.end(), 0) > 0) {
        log_and_throw_error("curve history: the collapses do not leave the simplified curves");
    }
    for (Eigen::Index v = 0; v < V_out.rows(); ++v) {
        compact.moves.push_back({uint64_t(v), point_of(V_out, v)});
    }
    h.edits.push_back(std::move(compact));
    return h;
}

} // namespace wmtk::history
