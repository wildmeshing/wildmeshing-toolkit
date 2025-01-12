
#pragma once
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategyBase.hpp>

namespace wmtk::components::mesh_info::simplex {

std::shared_ptr<wmtk::operations::AttributeTransferStrategyBase> dihedral_angles(
    const attribute::MeshAttributeHandle& mesh_attr,
    const attribute::MeshAttributeHandle& dihedral_angle_attr,
    bool run = true);

template <typename T>
std::shared_ptr<wmtk::operations::AttributeTransferStrategyBase> dihedral_angles(
    const attribute::MeshAttributeHandle& m,
    const std::string_view& name,
    bool run = true);

template <typename T>
std::shared_ptr<wmtk::operations::AttributeTransferStrategyBase> dihedral_angles(
    const attribute::MeshAttributeHandle& position,
    Mesh& mesh,
    const std::string_view& name,
    bool run = true);
} // namespace wmtk::components::mesh_info::simplex
