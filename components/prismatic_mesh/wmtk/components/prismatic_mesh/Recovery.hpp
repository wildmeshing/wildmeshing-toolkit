#pragma once

#include <wmtk/Types.hpp>

#include <array>
#include <vector>

namespace wmtk::components::prismatic_mesh {

// Paper Algorithm 9 classifies an offset triangle by the number of marked
// vertices. This structure contains only the resulting cell connectivity;
// geometric acceptance is deliberately handled by the recursive predicates in
// Validity.cpp.
struct PrismRecoveryPartition
{
    std::vector<std::array<size_t, 4>> tets;
    std::vector<std::array<size_t, 5>> pyramids;
    std::vector<std::array<size_t, 6>> prisms;
};

PrismRecoveryPartition recover_prism_partition(
    const std::array<size_t, 6>& prism,
    const std::array<bool, 3>& marked_offset_vertices);

} // namespace wmtk::components::prismatic_mesh
