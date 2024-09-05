#pragma once
#include <array>
#include <memory>
#include <vector>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>


namespace wmtk {
class Mesh;
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;
} // namespace wmtk

namespace wmtk::multimesh::utils {

/**
 * @brief extract a child mesh based on the given tag and tag value, and register it to the input
 * (parent) mesh
 *
 * @param m mesh
 * @param tag tag of type int64_t. The tag represents the child mesh that should be extracted.
 * @param tag_value target tag value
 * @param pt primitive type of the tag
 * @param child_is_free child mesh has a free topology
 * @return std::shared_ptr<Mesh> the shared pointer to the child mesh
 */
std::shared_ptr<Mesh> extract_and_register_child_mesh_from_tag(
    Mesh& m,
    const std::string& tag,
    const int64_t tag_value,
    const PrimitiveType pt,
    bool child_is_free = false);

/**
 * @brief extract a child mesh based on the tag handle and the tag value, and register it to the
 * input (parent) mesh
 *
 * @param m mesh
 * @param tag_handle attribute handle of the tag. The tag represents the child mesh that should be
 * extracted.
 * @param tag_value target tag value
 * @param child_is_free child mesh has a free topology
 * @return std::shared_ptr<Mesh>
 */
std::shared_ptr<Mesh> extract_and_register_child_mesh_from_tag_handle(
    Mesh& m,
    const wmtk::attribute::TypedAttributeHandle<int64_t>& tag_handle,
    const int64_t tag_value,
    bool child_is_free = false);


/**
 * @brief extract a child mesh based on the tag handle and the tag value, and register it to the
 * input (parent) mesh
 *
 * @param m mesh
 * @param tag_handle attribute handle of the tag. The tag represents the child mesh that should be
 * extracted.
 * @param tag_value target tag value
 * @param child_is_free child mesh has a free topology
 * @return std::shared_ptr<Mesh>
 */
std::shared_ptr<Mesh> extract_and_register_child_mesh_from_tag(
    wmtk::attribute::MeshAttributeHandle& tag_handle,
    const wmtk::attribute::MeshAttributeHandle::ValueVariant& tag_value,
    bool child_is_free = false);

} // namespace wmtk::multimesh::utils
