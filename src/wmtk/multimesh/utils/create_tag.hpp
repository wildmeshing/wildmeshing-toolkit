#pragma once
#include <set>
#include <wmtk/Mesh.hpp>
#include "internal/TupleTag.hpp"

namespace wmtk::multimesh::utils {
/**
 * @brief Do two passes on the edges of a triangle mesh to create vertex tags and edge tags.
 * First pass is to create vertex tags, and second pass is to create edge tags. The function return
 * a set of integers that are the valid tags.
 *
 * @param tuple_tag
 * @param critical_vids
 */
std::set<int64_t> create_tags(Mesh& m, const std::set<int64_t>& critical_vids);
} // namespace wmtk::multimesh::utils