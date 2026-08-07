#pragma once

#include <nlohmann/json.hpp>
#include <wmtk/Types.hpp>

#include <array>
#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace wmtk::components::prismatic_mesh {

struct ShellDefinition
{
    std::string group;
    double thickness = 0;
};

struct RodDefinition
{
    std::string group;
    double thickness = 0;
    std::vector<Vector2d> cross_section;
    size_t subdivisions = 0;
    double circular_blend = 0;
};

struct PrismaticMeshParameters
{
    std::vector<std::string> solid_groups;
    std::vector<ShellDefinition> shells;
    std::vector<RodDefinition> rods;
    size_t max_shell_iterations = 100;
    size_t validity_max_depth = 8;
    double jacobian_tolerance = 1e-12;
    bool use_topological_offset = true;
    std::filesystem::path report;
};

struct PrismaticMeshInput
{
    MatrixXd vertices;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<std::set<std::string>> tet_groups;
    std::map<std::string, std::vector<std::array<size_t, 3>>> face_groups;
    std::map<std::string, std::vector<std::array<size_t, 2>>> edge_groups;
};

struct HybridVolumeMesh
{
    MatrixXd vertices;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<std::array<size_t, 5>> pyramids;
    std::vector<std::array<size_t, 6>> prisms;
    std::vector<std::string> tet_region_tags;
    std::vector<std::string> pyramid_region_tags;
    std::vector<std::string> prism_region_tags;
};

struct PrismaticMeshReport
{
    size_t input_tets = 0;
    size_t solid_tets = 0;
    size_t shell_prisms = 0;
    size_t shell_pyramids = 0;
    size_t rod_prisms = 0;
    size_t fallback_tets = 0;
    size_t invalid_prism_candidates = 0;
    size_t shell_shrink_iterations = 0;
    size_t collision_clipped_shell_vertices = 0;
    size_t topological_offset_runs = 0;
    size_t topological_offset_tets = 0;
    size_t topological_mapped_vertices = 0;
    size_t topological_target_fallbacks = 0;
    size_t offset_operation_iterations = 0;
    size_t equal_source_collapse_attempts = 0;
    size_t equal_source_collapses = 0;
    size_t unlock_attempts = 0;
    size_t unlocks = 0;
    size_t unlock_rollbacks = 0;
    size_t remaining_equal_source_edges = 0;
    size_t remaining_tau_2_2 = 0;
    size_t open_boundary_edges = 0;
    size_t open_boundary_candidate_tets = 0;
    size_t open_boundary_removed_tets = 0;
    size_t marked_shell_sources = 0;
    size_t least_squares_shell_vertices = 0;
    size_t singular_shell_vertices = 0;
    size_t shell_smoothing_attempts = 0;
    size_t shell_smoothing_accepts = 0;
    size_t shell_smoothing_collision_rejects = 0;
    size_t shell_smoothing_iterations = 0;
    size_t solid_blocked_shell_sides = 0;
    size_t nonmanifold_shell_edges = 0;
    size_t shell_vertex_sectors = 0;
    size_t irregular_rod_vertices = 0;
    size_t rod_refined_segments = 0;
    size_t rod_refinement_max_depth = 0;
    size_t rod_joint_tets = 0;
    size_t rod_shell_junctions = 0;
    size_t rod_shell_components = 0;
    size_t rod_shell_transition_tets = 0;
    size_t duplicate_cells_removed = 0;
    double minimum_shell_scale = 1;
    double max_amips = 0;
    std::vector<std::string> warnings;

    nlohmann::json to_json() const;
};

struct PrismaticMeshResult
{
    HybridVolumeMesh mesh;
    PrismaticMeshReport report;
};

PrismaticMeshInput read_prismatic_msh(const std::filesystem::path& path);
void write_hybrid_msh(const std::filesystem::path& path, const HybridVolumeMesh& mesh);

PrismaticMeshParameters parameters_from_json(const nlohmann::json& json);

bool is_valid_cross_section(const std::vector<Vector2d>& polygon, std::string* reason = nullptr);

bool is_valid_tet(const std::array<Vector3d, 4>& points, double tolerance = 0);
bool is_valid_prism(
    const std::array<Vector3d, 6>& points,
    double tolerance = 1e-12,
    size_t max_depth = 8);
bool is_valid_pyramid(
    const std::array<Vector3d, 5>& points,
    double tolerance = 1e-12,
    size_t max_depth = 8);

PrismaticMeshResult generate_prismatic_mesh(
    const PrismaticMeshInput& input,
    const PrismaticMeshParameters& parameters);

} // namespace wmtk::components::prismatic_mesh
