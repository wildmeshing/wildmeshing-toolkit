#pragma once

#include <SimpleBVH/BVH.hpp>
#include <wmtk/Mesh.hpp>

namespace wmtk::components::internal::utils {

std::shared_ptr<SimpleBVH::BVH> bvh_from_mesh(
    const attribute::MeshAttributeHandle& position_handle);

std::shared_ptr<SimpleBVH::BVH> bvh_from_mesh(
    const attribute::MeshAttributeHandle& position_handle,
    const PrimitiveType pt);

/**
 * @brief Generate a bvh from tagged simplices.
 *
 * This method assumes that the bvh should contain triangles. If the labels are on tetrahedra, the
 * boundary of those is used.
 */
std::shared_ptr<SimpleBVH::BVH> bvh_from_mesh(
    attribute::MeshAttributeHandle& position_handle,
    attribute::MeshAttributeHandle& label_handle,
    const int64_t label_value);

} // namespace wmtk::components::internal::utils