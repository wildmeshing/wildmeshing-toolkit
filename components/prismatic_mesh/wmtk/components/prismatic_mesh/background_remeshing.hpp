#pragma once

#include "prismatic_mesh.hpp"

namespace wmtk::components::prismatic_mesh {

struct BackgroundCollapseGoal
{
    size_t removed;
    size_t survivor;
};

// Includes only directions passing the shell-volume, link and orphan checks, with at
// least one retained background tet that would become nonpositive after collapse.
std::vector<BackgroundCollapseGoal> background_blocked_collapses(
    const PrismaticMeshInput& input,
    double min_tet_volume);

void validate_background_remeshing_options(const BackgroundRemeshingOptions& options);
double background_tet_mean_ratio(const MatrixXd& vertices, const std::array<size_t, 4>& tet);

// Local 2-3/3-2 swaps and bounded interior-vertex relaxation. Input/band cells, all their
// vertices, and the active-domain boundary are fixed. Rejections roll back locally.
// Expects a compact, synchronized mesh. Returns diagnostics even when no edit is accepted.
nlohmann::json remesh_background(
    PrismaticMeshInput& input,
    double min_tet_volume,
    const BackgroundRemeshingOptions& options,
    std::vector<BackgroundCollapseGoal>* goals_before = nullptr);

} // namespace wmtk::components::prismatic_mesh
