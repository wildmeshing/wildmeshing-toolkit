#pragma once

#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/Accessor.hpp>

wmtk::Tuple get_edge_tuple(const wmtk::attribute::Accessor<int64_t, wmtk::TriMesh, 3>& FV, int64_t global_id,  const int64_t a, const int64_t b);
