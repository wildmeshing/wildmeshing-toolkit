#pragma once

#include <Eigen/Geometry>
#include <map>
#include <set>
#include "prismatic_mesh.hpp"

namespace wmtk::components::prismatic_mesh {

struct RodBackgroundRemeshingOptions
{
    bool enabled = false;
    double radius = 0; // Distance outside the actual rod surface; zero uses rod radius.
    double target_length = 0; // Zero uses rod radius.
    int passes = 2;
    double time_limit = 30;
    size_t max_splits = 2000;
};

struct RodOptions
{
    bool edge_based = false;
    double radius = 0; // Zero: thicknessratio times the mean centerline edge length.
    double thicknessratio = 0.1;
    int iterations = 5;
    int position_iterations = 20;
    double position_weight = 100;
    double laplace_weight = 10;
    double position_smoothing_weight = 0;
    int position_smoothing_iterations = 20;
    double position_smoothing_geometry_weight = 1000;
    int joint_smoothing_iterations = 0;
    double joint_smoothing_weight = 20;
    double joint_geometry_weight = 100;
    double joint_blend_radius_ratio = 0;
    int coupled_rounds = 0;
    int coupled_iterations = 20;
    double coupled_smoothing_weight = 10;
    double coupled_geometry_weight = 100;
    RodBackgroundRemeshingOptions background_remeshing;
    double min_tet_volume = 0;
};

struct RodFrame
{
    std::array<size_t, 2> edge;
    size_t edge_index = 0; // Incident centerline path edge.
    size_t center = 0;
    std::vector<size_t> ring;
    Vector3d tangent = Vector3d::Zero();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    size_t rod = 0;
    size_t order = 0;
    size_t initial_degree = 0;
    bool valid = false;
    std::string reason;
};

struct RodEdgeFrame
{
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    size_t degree = 0;
    bool from_link = false;
};

struct RodMeshState
{
    RodOptions options;
    double radius = 0;
    std::vector<std::array<size_t, 2>> input_edges;
    // Indexed like input_edges, directed along the containing rod's traversal.
    std::vector<std::array<size_t, 2>> directed_input_edges;
    std::vector<RodEdgeFrame> edge_frames;
    std::map<size_t, Eigen::Quaterniond> vertex_rotations;
    std::set<size_t> joints, endpoints, free_endpoints, protected_input;
    std::set<size_t> input_vertices, interface_vertices;
    std::set<size_t> retained_ring_vertices;
    std::map<size_t, Vector3d> frame_targets;
    std::map<size_t, Vector3d> joint_position_reference;
    std::set<size_t> frozen_offset;
    std::vector<std::vector<size_t>> rods; // Edges in path order; includes closed paths.
    std::vector<RodFrame> frames;
    std::map<size_t, size_t> vertex_frame;
    // A midpoint section is defined by the full link of this source half-edge.
    // It is not inferred from the topology of a correspondence region.
    std::map<size_t, std::array<size_t, 2>> section_edges;
    std::set<size_t> mapping_controls;
    // The only nonzero layer offset is the seam of a closed material mapping.
    std::map<std::array<size_t, 2>, size_t> layer_slot_shifts;
    bool mapping_locked = false;
    bool collapsing_regions = false;
    // Completed shell cells are immutable during the subsequent rod stage.
    std::set<size_t> fixed_shell_vertices;
    std::vector<std::pair<HybridCell, std::vector<std::array<size_t, 4>>>> fixed_shell_cells;
    std::set<std::array<size_t, 4>> fixed_shell_tets;
    nlohmann::json report;
};

// Equal-weight incident-edge pose mean. Edge-based degree-two planes use the
// geometric tangent bisector, independent of the incident rings' in-plane phases.
void average_rod_vertex_frames(RodMeshState& state);
// Ray-map a degree direction through edge -> averaged vertex -> edge planes.
// Snap only the terminal direction and distribute its phase residual by arc
// length (the closed-form length-weighted twist least-squares solution).
void transport_rod_frame_mapping(PrismaticMeshInput& input);
void transport_rod_section_mapping(PrismaticMeshInput& input);
void finalize_rod_frame_mapping(PrismaticMeshInput& input);
std::pair<std::vector<size_t>, std::vector<size_t>> rod_layer_rings(
    const PrismaticMeshInput& input,
    const std::array<size_t, 2>& axis);

// Collapse original-correspondence surface regions into vertex rings, equalize
// their degree along each rod, and assign transported material-slot indices.
// Input points and protected junction stars remain fixed. All geometry changes
// validate the supplied band/background tetrahedra.
void preprocess_rod_mesh(PrismaticMeshInput& input, const RodOptions& options);
void apply_rod_targets(PrismaticMeshInput& input);
void construct_rod_frame_topology(PrismaticMeshInput& input);
// With the degree and material mapping fixed, optimize the still-tetrahedral
// rod volume and its front. Native prisms have not been constructed at this point.
void optimize_rod_tet_positions(PrismaticMeshInput& input);
// Separate geometry-only solve for frozen pure-rod joint offsets. Keep input
// points and all tet connectivity; refresh protection positions after validation.
void optimize_rod_joint_positions(PrismaticMeshInput& input);
// Remesh a surface-distance neighborhood of pure background tets. Rod cells,
// rod vertices and the extracted neighborhood boundary remain fixed.
void remesh_rod_background(PrismaticMeshInput& input);
struct RodBackgroundNeighborhood
{
    std::set<size_t> movable_vertices;
    size_t selected_tets = 0;
    size_t boundary_vertices = 0;
    double radius = 0;
};
RodBackgroundNeighborhood rod_background_neighborhood(const PrismaticMeshInput& input);
// Alternate local background remeshing and a simultaneous rod/joint/background
// geometry solve before any native prism/pyramid reconstruction.
void optimize_rod_background_coupled(PrismaticMeshInput& input);
// Use a complete prism fan as a flat free end. Its exterior offset cap remains
// valid background; incomplete endpoint fans keep their original band cells.
void finalize_rod_ends(PrismaticMeshInput& input, PrismDominantMesh& hybrid);
bool rod_collapse_allowed(const PrismaticMeshInput& input, size_t removed, size_t survivor);
bool is_rod_offset_vertex(const PrismaticMeshInput& input, size_t vertex);
bool is_rod_band_tet(const PrismaticMeshInput& input, size_t tid);
bool is_fixed_shell_tet(const PrismaticMeshInput& input, size_t tid);
bool has_shell_region(const PrismaticMeshInput& input);
void reserve_shell_cells(PrismaticMeshInput& input, const PrismDominantMesh& hybrid);
void restore_shell_cells(const PrismaticMeshInput& input, PrismDominantMesh& hybrid);
nlohmann::json analyze_rod_tau(const PrismaticMeshInput& input);

// Insert missing ring-to-ring surface edges by local swaps of both incident
// volume stars. Complete layer repairs are committed together; failed repairs
// restore the original layer. Vertex coordinates and correspondence stay fixed.
nlohmann::json align_rod_interfaces_by_swaps(PrismaticMeshInput& input, double min_tet_volume);

} // namespace wmtk::components::prismatic_mesh
