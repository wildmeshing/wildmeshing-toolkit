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
    std::set<size_t> frozen_offset;
    std::vector<std::vector<size_t>> rods; // Edges in path order; includes closed paths.
    std::vector<RodFrame> frames;
    std::map<size_t, size_t> vertex_frame;
    bool collapsing_regions = false;
    nlohmann::json report;
};

// Equal-weight incident-edge frame mean: one edge copies its pose, two edges
// use their quaternion midpoint, and higher valence uses a quaternion mean.
void average_rod_vertex_frames(RodMeshState& state);
// Ray-map a degree direction through edge -> averaged vertex -> edge planes.
// Snap only the terminal direction and distribute its phase residual by arc
// length (the closed-form length-weighted twist least-squares solution).
void transport_rod_frame_mapping(PrismaticMeshInput& input);

// Collapse original-correspondence surface regions into vertex rings, equalize
// their degree along each rod, and assign transported material-slot indices.
// Input points and protected junction stars remain fixed. All geometry changes
// validate the supplied band/background tetrahedra.
void preprocess_rod_mesh(PrismaticMeshInput& input, const RodOptions& options);
void apply_rod_targets(PrismaticMeshInput& input);
void construct_rod_frame_topology(PrismaticMeshInput& input);
// Use a complete prism fan as a flat free end. Its exterior offset cap remains
// valid background; incomplete endpoint fans keep their original band cells.
void finalize_rod_ends(PrismaticMeshInput& input, PrismDominantMesh& hybrid);
bool rod_collapse_allowed(const PrismaticMeshInput& input, size_t removed, size_t survivor);
bool is_rod_offset_vertex(const PrismaticMeshInput& input, size_t vertex);
nlohmann::json analyze_rod_tau(const PrismaticMeshInput& input);

// Insert missing ring-to-ring surface edges by local swaps of both incident
// volume stars. Complete layer repairs are committed together; failed repairs
// restore the original layer. Vertex coordinates and correspondence stay fixed.
nlohmann::json align_rod_interfaces_by_swaps(PrismaticMeshInput& input, double min_tet_volume);

} // namespace wmtk::components::prismatic_mesh
