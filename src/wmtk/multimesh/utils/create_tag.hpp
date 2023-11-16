#pragma once
#include <set>
#include <wmtk/Mesh.hpp>
#include "internal/TupleTag.hpp"

namespace wmtk::multimesh::utils {
/**
 * @brief Go through edges of the parent mesh (triangle mesh) and initialize all the vertex tags to
 * be -1. It will create a TupleTag object and store it in the mesh as an attribute.
 *
 * @param m
 * @param critical_vids
 */
internal::TupleTag initialize_tags(Mesh& m, const std::set<long>& critical_vids);

/**
 * @brief Do two passes on the edges of a triangle mesh to create vertex tags and edge tags.
 * First pass is to create vertex tags, and second pass is to create edge tags.
 *
 * @param tuple_tag
 * @param critical_vids
 */
void create_tags(internal::TupleTag& tuple_tag, const std::set<long>& critical_vids);
} // namespace wmtk::multimesh::utils