#pragma once

#include <wmtk/operations/AttributesUpdate.hpp>

#include <memory>

namespace SimpleBVH {
class BVH;
}

namespace wmtk::operations::composite {
class ProjectOperation : public AttributesUpdate
{
public:
    ProjectOperation(
        std::shared_ptr<Operation> main_op,
        const attribute::MeshAttributeHandle& project_to_mesh,
        Mesh& m,
        attribute::MeshAttributeHandle& child_mesh_coordinates);

    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;
    PrimitiveType primitive_type() const override { return m_main_op->primitive_type(); }


private:
    const std::shared_ptr<wmtk::operations::Operation> m_main_op;
    const TypedAttributeHandle<double> m_coordinates;
    Mesh& m_child_mesh;
    std::shared_ptr<SimpleBVH::BVH> m_bvh = nullptr;
};
} // namespace wmtk::operations::composite
