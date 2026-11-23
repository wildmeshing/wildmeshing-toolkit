#pragma once

#include "prismatic_mesh.hpp"

namespace wmtk::components::prismatic_mesh {

struct QualityImprovementOptions
{
    int cycles = 4;
    int mixed_smoothing_sweeps = 120;
    int volume_passes = 2;
    size_t column_collapse_attempts = 100;
    size_t surface_split_budget = 1200;
    size_t surface_flip_attempts = 1200;
    size_t volume_split_budget = 12000;
    double time_limit = 900;
    double initial_sizing_factor = 0.25;
    double quality_target = 20;
    double smoothing_target = 18;
    double thickness_weight = 10;
    bool topology_enabled = true;

    static QualityImprovementOptions from_json(const nlohmann::json& options);
    nlohmann::json json() const;
    void validate() const;
};

struct ImprovedHybridCell
{
    HybridCellType type;
    std::vector<size_t> vertices; // VTK ordering; the native prism reference reverses each cap
    int region = 3; // 1: input volume, 2: shell, 3: background
    double amips = 0; // raw AMIPS; ideal = 3; native order-4 prism/pyramid integral
};

// Retopology has its own vertex/cell row space. The original hybrid's reference
// tetrahedra describe the construction, not this result, and are left untouched.
struct ImprovedHybridMesh
{
    MatrixXd vertices;
    std::vector<ImprovedHybridCell> cells;
    std::vector<int64_t> source_vertex_ids;
    std::vector<int64_t> correspondence; // rows in vertices; negative means geometric/free
    std::vector<int64_t> original_vertex_rows; // -1 for inserted vertices
    std::vector<int> vertex_tags;
    MatrixXd reference_base;
    MatrixXd reference_normals;
    std::vector<double> target_heights; // zero where no active reference-height constraint exists
    std::vector<double> target_edge_lengths;
    nlohmann::json report;
};

// Does not mutate the construction/reference mesh. Preserves material interfaces,
// retained input geometry, positive actual cells, and the exterior domain. The
// returned checkpoint minimizes the global maximum AMIPS over valid stages.
ImprovedHybridMesh improve_prismatic_mesh_quality(
    const PrismaticMeshInput& input,
    const PrismDominantMesh& hybrid,
    const QualityImprovementOptions& options = {});

void write_improved_prismatic_mesh(
    const ImprovedHybridMesh& mesh,
    const std::filesystem::path& path,
    bool include_background = true,
    MeshMap* output = nullptr,
    nlohmann::json* report = nullptr);

} // namespace wmtk::components::prismatic_mesh
