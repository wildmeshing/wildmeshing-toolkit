#pragma once

#include "prismatic_mesh.hpp"

namespace wmtk::components::prismatic_mesh {

// Worst AMIPS of the native linear wedge map, relative to an equilateral unit prism.
// Ideal = 3. Includes stationary points along the three reference column edges.
double prism_amips(const MatrixXd& vertices, const HybridCell& prism);

// Final geometry-only pass. Fit shell normal distance and rod section radius/plane,
// and improve native prism shape. Other cells participate only in validity constraints.
// Input, protected offsets, material correspondence and all connectivity stay fixed.
void smooth_thickness_and_prisms(
    PrismaticMeshInput& input,
    PrismDominantMesh& hybrid,
    double min_tet_volume,
    int iterations);

} // namespace wmtk::components::prismatic_mesh
