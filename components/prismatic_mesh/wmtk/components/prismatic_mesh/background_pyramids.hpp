#pragma once

#include <algorithm>
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
    struct SequentialStep
    {
        BackgroundPyramidPlan plan;
        size_t point_vertex_row;
        size_t layer;
    };
    // Only overlap failures are retried on a private, updated tet mesh. These
    // steps follow the original disjoint batch and are replayed at final commit.
    std::vector<SequentialStep> sequential;
    size_t planning_tet_count = 0;
    bool contains(const std::array<size_t, 4>& quad) const
    {
        if (accepted.count(quad)) return true;
        return std::any_of(sequential.begin(), sequential.end(), [&](const auto& step) {
            auto q = step.plan.quad.vertices;
            std::sort(q.begin(), q.end());
            return q == quad;
        });
    }
};

// Conservative, disjoint background edge-star cavities. Optional bounded growth
// includes background face neighbors if the initial kernel is empty. Rejections
// leave the original triangle interface available as a conforming fallback.
BackgroundPyramidPlans plan_background_pyramids(
    const PrismaticMeshInput& input,
    const std::vector<BackgroundQuad>& quads,
    double min_volume,
    int max_cavity_growth = 0,
    bool sequential_retry = true);

// Commit all exterior pyramid cavities of a rod layer together. Earlier accepted
// layers reserve disjoint cavities; overlap failures additionally get a sequential
// retry on a private updated mesh, protecting emitted pyramid bodies. Failed
// retries roll back the entire layer. The caller still commits all plans together.
BackgroundPyramidPlans plan_atomic_rod_layer_pyramids(
    const PrismaticMeshInput& input,
    const std::vector<std::vector<BackgroundQuad>>& layers,
    double min_volume,
    std::vector<std::string>& reasons,
    int max_cavity_growth = 2);

// Apply only plans whose quad is emitted by a shell hybrid cell. Existing non-cavity
// reference row IDs stay fixed. New background cells own the reconstructed reference tets.
void apply_background_pyramids(
    PrismaticMeshInput& reference,
    PrismDominantMesh& hybrid,
    BackgroundPyramidPlans& plans);

} // namespace wmtk::components::prismatic_mesh::detail
