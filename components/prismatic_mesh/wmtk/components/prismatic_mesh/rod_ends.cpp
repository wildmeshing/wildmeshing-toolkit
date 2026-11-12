#include "prism_jacobian.hpp"
#include "rod_mesh.hpp"

#include <algorithm>
#include <cmath>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {
namespace {
using Tri = std::array<size_t, 3>;
template <class T>
T sorted(T value)
{
    std::sort(value.begin(), value.end());
    return value;
}
void require(bool valid, const std::string& reason)
{
    if (!valid) throw std::runtime_error("rod ends: " + reason);
}
Vector3d position(const PrismaticMeshInput& input, size_t v)
{
    return input.vertices.row(v).transpose();
}
std::vector<std::vector<size_t>> hybrid_faces(const HybridCell& c)
{
    const auto& v = c.vertices;
    if (c.type == HybridCellType::Tetrahedron)
        return {{v[1], v[2], v[3]}, {v[0], v[3], v[2]}, {v[0], v[1], v[3]}, {v[0], v[2], v[1]}};
    if (c.type == HybridCellType::Pyramid)
        return {
            {v[0], v[3], v[2], v[1]},
            {v[0], v[1], v[4]},
            {v[1], v[2], v[4]},
            {v[2], v[3], v[4]},
            {v[3], v[0], v[4]}};
    return {
        {v[0], v[2], v[1]},
        {v[3], v[4], v[5]},
        {v[0], v[1], v[4], v[3]},
        {v[1], v[2], v[5], v[4]},
        {v[2], v[0], v[3], v[5]}};
}

} // namespace

void finalize_rod_ends(PrismaticMeshInput& input, PrismDominantMesh& hybrid)
{
    require(input.rod_state != nullptr, "missing rod state");
    auto& state = *input.rod_state;
    state.report["terminal_frames"] = nlohmann::json::array();
    if (state.free_endpoints.empty()) return;
    const auto parent = [&](const auto& vertices) -> int64_t {
        int64_t common = -1;
        for (size_t v : vertices) {
            const int64_t p = input.vertex_tags[v] == 1   ? static_cast<int64_t>(v)
                              : input.vertex_tags[v] == 2 ? input.corr_input_vertex[v]
                                                          : -1;
            if (p < 0 || !state.free_endpoints.count(p) || (common >= 0 && common != p)) return -1;
            common = p;
        }
        return common;
    };
    // Keep only faces in a free endpoint's fiber. The complete background still
    // contributes their other owners, without storing unrelated far faces.
    std::map<Tri, std::vector<size_t>> owners;
    for (size_t ci = 0; ci < hybrid.cells.size(); ++ci)
        for (const auto& f : hybrid_faces(hybrid.cells[ci]))
            if (f.size() == 3 && parent(f) >= 0)
                owners[sorted(Tri{f[0], f[1], f[2]})].push_back(ci);
    std::set<size_t> returned_cells;
    size_t completed = 0;
    for (size_t v : state.free_endpoints) {
        nlohmann::json report = {
            {"center_vid", input.source_vertex_ids[v]},
            {"definition", "material_frame_disk_through_input_endpoint"}};
        const auto fail = [&](const char* reason) {
            report["status"] = reason;
            state.report["terminal_frames"].push_back(report);
        };
        if (!state.vertex_frame.count(v)) {
            fail("missing_terminal_frame");
            continue;
        }
        const auto& frame = state.frames.at(state.vertex_frame.at(v));
        const Vector3d center = position(input, v);
        const double tolerance = 1e-8 * state.radius;
        bool valid = frame.valid && frame.ring.size() >= 3;
        for (size_t w : frame.ring)
            valid &= std::abs((position(input, w) - center).dot(frame.tangent)) <= tolerance;
        if (!valid) {
            fail("terminal_frame_not_planar");
            continue;
        }
        const auto is_band = [&](size_t ci) {
            return input.offset_tet_tags[hybrid.cells[ci].source_tets.front()] == 1;
        };
        const auto is_cap = [&](size_t ci) {
            const auto& cell = hybrid.cells[ci];
            if (!is_band(ci) || cell.type != HybridCellType::Tetrahedron ||
                parent(cell.vertices) != static_cast<int64_t>(v))
                return false;
            bool outside = false;
            const auto& edge = state.directed_input_edges[frame.edge_index];
            const Vector3d outward = edge[0] == v ? -frame.tangent : frame.tangent;
            for (size_t w : cell.vertices) {
                const double d = (position(input, w) - center).dot(outward);
                if (d < -tolerance) return false;
                outside |= d > tolerance;
            }
            return outside;
        };
        std::set<Tri> disk;
        std::set<size_t> prisms, cap;
        std::vector<size_t> pending;
        for (size_t k = 0; k < frame.ring.size() && valid; ++k) {
            const Tri face = sorted(Tri{v, frame.ring[k], frame.ring[(k + 1) % frame.ring.size()]});
            disk.insert(face);
            const auto found = owners.find(face);
            if (found == owners.end() || found->second.empty() || found->second.size() > 2) {
                valid = false;
                break;
            }
            size_t prism_count = 0;
            for (size_t ci : found->second) {
                const auto& cell = hybrid.cells[ci];
                if (is_band(ci) && cell.type == HybridCellType::Prism &&
                    minimum_prism_jacobian(input.vertices, cell).determinant > 0) {
                    prisms.insert(ci);
                    ++prism_count;
                } else if (is_cap(ci))
                    pending.push_back(ci);
                else if (is_band(ci))
                    valid = false;
            }
            valid &= prism_count == 1;
        }
        if (!valid) {
            fail("unmatched_terminal_prism_sector");
            continue;
        }
        while (!pending.empty() && valid) {
            const size_t ci = pending.back();
            pending.pop_back();
            if (!cap.insert(ci).second) continue;
            for (const auto& f : hybrid_faces(hybrid.cells[ci])) {
                const Tri key = sorted(Tri{f[0], f[1], f[2]});
                if (disk.count(key)) continue;
                for (size_t neighbor : owners.at(key)) {
                    if (neighbor == ci) continue;
                    if (is_cap(neighbor))
                        pending.push_back(neighbor);
                    else if (is_band(neighbor))
                        valid = false;
                }
            }
        }
        if (!valid) {
            fail("offset_cap_has_another_band_interface");
            continue;
        }
        // The complete material frame defines the rod boundary. Its disk is
        // covered by real prism caps; exterior cone tets remain background tets.
        returned_cells.insert(cap.begin(), cap.end());
        report["status"] = "prismatic_flat_end";
        report["degree"] = frame.ring.size();
        report["prism_cells"] = prisms;
        report["offset_cap_cells_returned_to_background"] = cap;
        report["ring_vids"] = nlohmann::json::array();
        for (size_t w : frame.ring) report["ring_vids"].push_back(input.source_vertex_ids[w]);
        state.report["terminal_frames"].push_back(std::move(report));
        ++completed;
    }
    const auto old_tags = input.offset_tet_tags;
    for (size_t ci : returned_cells)
        for (size_t tid : hybrid.cells[ci].source_tets) input.offset_tet_tags[tid] = -1;
    try {
        validate_prism_dominant_mesh(input, hybrid, state.options.min_tet_volume);
    } catch (...) {
        input.offset_tet_tags = old_tags;
        throw;
    }
    size_t band_tets = 0, background_tets = 0;
    for (const auto& cell : hybrid.cells)
        if (cell.type == HybridCellType::Tetrahedron) {
            const size_t tid = cell.source_tets.front();
            band_tets += input.offset_tet_tags[tid] == 1;
            background_tets += input.offset_tet_tags[tid] != 1 && input.input_cells[tid] != 1;
        }
    hybrid.report["band_tetrahedra"] = band_tets;
    hybrid.report["background_tetrahedra"] = background_tets;
    state.report["completed_prismatic_free_ends"] = completed;
    state.report["offset_cap_tets_returned_to_background"] = returned_cells.size();
    hybrid.report["rod_terminal_frames"] = state.report["terminal_frames"];
    hybrid.report["checks"]["free_end_frame_disks_checked"] = true;
    logger().info(
        "Rod terminal frames: {}/{} prismatic ends; {} offset cap tets returned to background",
        completed,
        state.free_endpoints.size(),
        returned_cells.size());
}


} // namespace wmtk::components::prismatic_mesh
