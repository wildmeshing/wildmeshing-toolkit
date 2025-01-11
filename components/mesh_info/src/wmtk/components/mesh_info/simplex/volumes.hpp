#pragma once
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategyBase.hpp>

namespace wmtk::components::mesh_info::simplex {

std::shared_ptr<wmtk::operations::AttributeTransferStrategyBase> volumes(
    const attribute::MeshAttributeHandle& mesh_attr,
    const attribute::MeshAttributeHandle& volume_attr,
    bool run = true);

template <typename T>
std::shared_ptr<wmtk::operations::AttributeTransferStrategyBase> volumes(
    const attribute::MeshAttributeHandle& m,
    wmtk::PrimitiveType primitive_type,
    const std::string_view& name,
    bool run = true);

template <typename T>
std::shared_ptr<wmtk::operations::AttributeTransferStrategyBase> volumes(
    const attribute::MeshAttributeHandle& position,
    Mesh& mesh,
    wmtk::PrimitiveType primitive_type,
    const std::string_view& name,
    bool run = true);
} // namespace wmtk::components::mesh_info::simplex
