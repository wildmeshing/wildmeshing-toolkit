#pragma once

#include <Eigen/Geometry>
#include <map>
#include <set>
#include "prismatic_mesh.hpp"

namespace wmtk::components::prismatic_mesh {

struct RodOptions
{
    double radius = 0; // Zero: thicknessratio times the mean centerline edge length.
    double thicknessratio = 0.1;
    int iterations = 5;
    double min_tet_volume = 0;
    bool axial_split = true;
    bool unified_optimization = false;
};

struct RodFrame
{
    std::array<size_t, 2> edge;
    size_t edge_index = 0; // Owning path edge; terminal cap sections may add a frame.
    bool terminal_section = false;
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
    std::set<size_t> frozen_offset;
    std::set<size_t> degree_blocked_centers;
    std::vector<std::vector<size_t>> rods; // Edges in path order; includes closed paths.
    std::vector<RodFrame> frames;
    std::map<size_t, size_t> vertex_frame;
    std::vector<std::array<size_t, 6>> columns;
    std::set<size_t> terminal_frame_vertices;
    std::set<size_t> terminal_prism_cells;
    nlohmann::json report;
};

// Equal-weight incident-edge frame mean: one edge copies its pose, two edges
// use their quaternion midpoint, and higher valence uses a quaternion mean.
void average_rod_vertex_frames(RodMeshState& state);

// All topology/geometry operations include the supplied background. Invalid local
// operations leave their original tetrahedra available as a conforming fallback.
void prepare_rod_mesh(PrismaticMeshInput& input, const RodOptions& options);
// Mixed-dimensional path: topology/frame preparation only, followed by the shared
// target/collapse/smoothing pipeline on the original construction mesh.
void preprocess_rod_mesh(PrismaticMeshInput& input, RodOptions options);
void apply_rod_targets(PrismaticMeshInput& input);
void refresh_rod_prism_candidates(PrismaticMeshInput& input);
// Normalize redundant regular-vertex fibers by checked split/collapse operations,
// then rebuild/equalize the actual edge-coface rings before column reconstruction.
void construct_rod_frame_topology(PrismaticMeshInput& input);
bool rod_collapse_allowed(const PrismaticMeshInput& input, size_t removed, size_t survivor);
bool is_rod_offset_vertex(const PrismaticMeshInput& input, size_t vertex);
nlohmann::json analyze_rod_tau(const PrismaticMeshInput& input);
// Insert complete midpoint frames into realizable hybrid intervals, creating the new
// input vertices, correspondence, cells and background interfaces in one transaction.
void split_rod_intervals(PrismaticMeshInput& input, PrismDominantMesh& hybrid);
// End a free rod on its material-frame disk through the input endpoint. Check
// each disk sector against a prism and return exterior offset caps to background.
void finalize_rod_terminal_frames(PrismaticMeshInput& input, PrismDominantMesh& hybrid);

struct RodCornerBuffer
{
    PrismaticMeshInput input;
    PrismDominantMesh hybrid;
};
// Construct an isolated, validated patch, keeping the corner pose and both outer
// section interfaces fixed. This does not replace the source background cavity.
// Failed geometric attempts double the subdivision up to the requested depth;
// exhausted requests return their original valid local tets, without source edits.
RodCornerBuffer construct_rod_corner_buffer(
    const PrismaticMeshInput& source,
    int64_t corner_source_vid,
    double max_frame_angle_degrees = 15,
    int max_subdivision_depth = 3);

} // namespace wmtk::components::prismatic_mesh
