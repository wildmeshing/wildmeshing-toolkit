#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>

namespace wmtk {
class Mesh;
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;
} // namespace wmtk

namespace wmtk::multimesh::utils {

/**
 * @brief extract and register a child mesh from a tag
 *
 * @param m  mesh
 * @param tag  should be a long type tag
 * @param tag_value  value of the target tag
 * @param pt  the primitive type of the tagged simplex
 */
void extract_and_register_child_mesh_from_tag(
    TriMesh& m,
    const std::string& tag,
    const long& tag_value,
    const PrimitiveType& pt);

} // namespace wmtk::multimesh::utils
