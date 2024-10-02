#pragma once

#include <string>
#include "WildmeshingOptions.hpp"

namespace wmtk::components::internal {

std::vector<std::pair<std::shared_ptr<Mesh>, std::string>> wildmeshing2d(
    const WildMeshingOptions& options);

// void adjust_sizing_field(
//     Mesh& m,
//     const TypedAttributeHandle<Rational>& coordinate_handle,
//     const TypedAttributeHandle<double>& edge_length_handle,
//     const TypedAttributeHandle<double>& sizing_field_scalar_handle,
//     const TypedAttributeHandle<double>& energy_handle,
//     const TypedAttributeHandle<double>& target_edge_length_handle,
//     const TypedAttributeHandle<char>& visited_handle,
//     const double stop_energy,
//     const double current_max_energy,
//     const double initial_target_edge_length,
//     const double min_target_edge_length);

// void set_operation_energy_filter(
//     Mesh& m,
//     const TypedAttributeHandle<Rational>& coordinate_handle,
//     const TypedAttributeHandle<double>& energy_handle,
//     const TypedAttributeHandle<char>& energy_filter_handle,
//     const TypedAttributeHandle<char>& visited_handle,
//     const double stop_energy,
//     const double current_max_energy,
//     const double initial_target_edge_length);

// void set_operation_energy_filter_after_sizing_field(
//     Mesh& m,
//     const TypedAttributeHandle<Rational>& coordinate_handle,
//     const TypedAttributeHandle<double>& energy_handle,
//     const TypedAttributeHandle<char>& energy_filter_handle,
//     const TypedAttributeHandle<char>& visited_handle,
//     const double stop_energy,
//     const double current_max_energy,
//     const double initial_target_edge_length);

} // namespace wmtk::components::internal