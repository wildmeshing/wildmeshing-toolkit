#pragma once
#include <cstdint>
#include <vector>
#include <wmtk/Tuple.hpp>
#include "utils.hpp"

std::vector<wmtk::Tuple>
boundary_edges_to_tuples(const EigenMeshes& em, const std::vector<int64_t>& t, bool offset = false);
