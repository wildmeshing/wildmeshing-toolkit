#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components::internal {

void periodic_optimization(
    Mesh& periodic_mesh,
    Mesh& position_mesh,
    Mesh& surface_mesh,
    const double target_edge_length,
    const double max_amips_energy,
    const int64_t passes,
    const double envelope_size,
    const bool intermediate_output,
    std::vector<attribute::MeshAttributeHandle>& pass_through_attributes,
    std::string output_dir,
    std::string output);

} // namespace wmtk::components::internal
