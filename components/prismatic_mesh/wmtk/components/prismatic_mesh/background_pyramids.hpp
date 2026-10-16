#pragma once

#include <array>
#include <map>
#include "prismatic_mesh.hpp"

namespace wmtk::components::prismatic_mesh::detail {

struct BackgroundQuad
{
    std::array<size_t, 4> vertices; // outward shell cycle, into the background
    std::array<std::array<size_t, 3>, 2> triangles; // existing interface diagonal
};

struct BackgroundPyramidPlan
{
    BackgroundQuad quad;
    std::vector<size_t> cavity;
    std::vector<std::array<size_t, 3>> boundary; // outward cavity triangles
    Vector3d point;
    double margin;
};

struct BackgroundPyramidPlans
{
    std::map<std::array<size_t, 4>, BackgroundPyramidPlan> accepted;
    nlohmann::json report;
};

// Conservative, disjoint background edge-star cavities. Rejections leave the original
// triangle interface available to the caller's ordinary conforming fallback.
BackgroundPyramidPlans plan_background_pyramids(
    const PrismaticMeshInput& input,
    const std::vector<BackgroundQuad>& quads,
    double min_volume);

// Apply only plans whose quad is emitted by a shell hybrid cell. Existing non-cavity
// reference row IDs stay fixed. New background cells own the reconstructed reference tets.
void apply_background_pyramids(
    PrismaticMeshInput& reference,
    PrismDominantMesh& hybrid,
    BackgroundPyramidPlans& plans);

} // namespace wmtk::components::prismatic_mesh::detail
