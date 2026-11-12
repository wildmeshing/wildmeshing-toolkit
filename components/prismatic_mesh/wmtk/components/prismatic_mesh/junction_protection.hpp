#pragma once

#include <map>
#include <set>
#include "prismatic_mesh.hpp"

namespace wmtk::components::prismatic_mesh {

struct JunctionProtection
{
    std::set<size_t> rod_shell_vertices;
    std::set<size_t> rod_end_vertices;
    std::set<size_t> rod_free_end_vertices;
    std::set<size_t> rod_joint_vertices;
    std::set<size_t> open_boundary_vertices;
    std::set<size_t> shell_curve_end_vertices;
    std::set<size_t> shell_terminal_vertices;
    std::set<size_t> centers;
    std::set<size_t> layered_centers;
    std::set<size_t> offset_vertices;
    std::map<size_t, Vector3d> offset_positions;
    std::vector<std::array<size_t, 4>> original_band_cells;
    bool collapse_connectivity_verified = false;
};

// Capture endpoint stars before topology changes. Protect shell-intersection
// curve endpoints only off the input open boundary, plus rod/non-rod attachments
// and branching rod joints. Free rod ends and unconnected loops are not protected.
void initialize_junction_protection(PrismaticMeshInput& input);
bool junction_tet_protected(const PrismaticMeshInput& input, const std::array<size_t, 4>& tet);
bool junction_collapse_allowed(const PrismaticMeshInput& input, size_t removed);
void verify_junction_one_rings(PrismaticMeshInput& input);
void apply_junction_targets(PrismaticMeshInput& input);
nlohmann::json junction_protection_report(const PrismaticMeshInput& input);

// Relax inward dents on the physical band boundary after hybrid reconstruction.
// Topology, input positions and completed rod-prism frames remain fixed. Each
// accepted move preserves reference tets and native prism/pyramid Jacobians.
void smooth_transition_regions(
    PrismaticMeshInput& input,
    PrismDominantMesh& hybrid,
    double min_tet_volume,
    int iterations);

} // namespace wmtk::components::prismatic_mesh
