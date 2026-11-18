#pragma once
#include "prismatic_mesh.hpp"

namespace wmtk::components::prismatic_mesh {
struct BadQualityRelocationOptions
{
    int collapse_edge_iterations = 0;
    int prism_tet_iterations = 0;
    int tet_face_iterations = 0;
    bool tet_face_allow_mixed_cells = true;
    double amips_threshold = 100;
    double thickness_weight = 1000;
    double thickness_tolerance = .1; // thickness stages: relative projected-thickness error
    double max_displacement_ratio = .5; // thickness stages: displacement / target thickness
    int max_backtracks = 40;
};

struct BadQualityRegions
{
    std::vector<size_t> collapse_edge_tets;
    std::vector<std::array<size_t, 2>> collapse_edges;
    std::vector<size_t> prism_tets;
    std::vector<size_t> other_tets;
};

// Classify actual shell tetrahedra, not prism/pyramid reference decompositions.
// Residual same-component/same-correspondence offset edges have priority; the
// second class comes from Jacobian-valid prism candidates emitted as tets.
BadQualityRegions classify_bad_quality_regions(
    const PrismaticMeshInput& input,
    const PrismDominantMesh& hybrid,
    double amips_threshold = 100);

// Geometry-only repair after hybrid construction. Relocate only the two residual
// edge ends in a shared offset-triangle chart using pulled-back tet AMIPS gradients.
// The domain boundary and UV triangle-collapse events bound a halving line search.
// This first class does not use target positions, thickness or a thickness-scaled
// displacement cap. Accept strictly decreasing total incident AMIPS without per-cell
// quality limits; domain orientation and all affected cells must remain positive.
// Then relocate bad prism-derived offset corners with incident AMIPS + projected-thickness
// energy, with bounds measured after the first class. Only this second class bounds
// each affected tet by max(threshold, its entry AMIPS), tightened after improvement.
// An optional final pass selects all offset-surface vertices incident to actual shell
// tets, including vertices incident to native prisms/pyramids when allowed by options.
// It minimizes incident AMIPS + projected-thickness energy without individual quality caps.
// Input/rod/fixed-background vertices and connectivity stay fixed. Junction
// no-collapse offsets may relocate while retaining their topology protection.
void relocate_bad_quality_regions(
    PrismaticMeshInput& input,
    PrismDominantMesh& hybrid,
    double min_tet_volume,
    const BadQualityRelocationOptions& options = {});

// After relocation, grow the transition marks by one offset-triangle vertex ring
// around current bad actual shell tets, rebuild hybrid cells, then repeat the
// configured relocation stages. Each round freezes its seeds before marking
// neighboring faces. Old marks persist; vertex labels and correspondence do not
// change. Zero rounds disables the retry; the component exposes this as opt-in.
void expand_bad_tet_transition_regions(
    PrismaticMeshInput& input,
    PrismDominantMesh& hybrid,
    double min_tet_volume,
    const BadQualityRelocationOptions& options,
    int rounds);
} // namespace wmtk::components::prismatic_mesh
