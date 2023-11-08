#pragma once
#include <array>
#include <memory>
#include <vector>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/MeshAttributes.hpp>


namespace wmtk {
class Mesh;
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;
} // namespace wmtk

namespace wmtk::multimesh::utils {

/**
 * @brief
 *
 * @param m mesh
 * @param tag tag of type long
 * @param tag_value target tag value
 * @param pt primitive type of the tag
 * @return std::shared_ptr<Mesh> the shared pointer to the child mesh
 */
std::shared_ptr<Mesh> extract_and_register_child_mesh_from_tag(
    Mesh& m,
    const std::string& tag,
    const long tag_value,
    const PrimitiveType& pt);

/**
 * @brief
 *
 * @param m mesh
 * @param tag_handle attribute handle of the tag
 * @param tag_value target tag value
 * @param pt primitive type of the tag
 * @return std::shared_ptr<Mesh>
 */
std::shared_ptr<Mesh> extract_and_register_child_mesh_from_tag_handle(
    Mesh& m,
    const MeshAttributeHandle<long>& tag_handle,
    const long tag_value,
    const PrimitiveType& pt);


} // namespace wmtk::multimesh::utils
