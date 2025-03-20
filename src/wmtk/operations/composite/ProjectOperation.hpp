#pragma once

#include <wmtk/operations/AttributesUpdate.hpp>

#include <memory>

namespace SimpleBVH {
class BVH;
}

namespace wmtk::submesh {
class Embedding;
}

namespace wmtk::operations::composite {
class ProjectOperation : public AttributesUpdate
{
public:
    // first: construction, second: query
    using MeshConstrainPair =
        std::pair<attribute::MeshAttributeHandle, attribute::MeshAttributeHandle>;

    ProjectOperation(
        std::shared_ptr<Operation> main_op,
        const attribute::MeshAttributeHandle& project_to_mesh,
        attribute::MeshAttributeHandle& child_mesh_coordinates);

    ProjectOperation(
        std::shared_ptr<Operation> main_op,
        const std::vector<MeshConstrainPair>& mesh_constaint_pairs);

    ProjectOperation(Mesh& mesh, const std::vector<MeshConstrainPair>& mesh_constaint_pairs);

    ProjectOperation(
        std::shared_ptr<Operation> main_op,
        const submesh::Embedding& emb,
        const attribute::MeshAttributeHandle& pos_handle);

    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override;
    PrimitiveType primitive_type() const override { return PrimitiveType::Vertex; }


private:
    using BVHConstrainPair =
        std::pair<attribute::MeshAttributeHandle, std::shared_ptr<SimpleBVH::BVH>>;

    std::shared_ptr<wmtk::operations::Operation> m_main_op;
    std::vector<BVHConstrainPair> m_bvh;
};
} // namespace wmtk::operations::composite
