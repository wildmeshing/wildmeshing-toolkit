#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TetMeshOperation.hpp"
#include "VertexAttributesUpdateBase.hpp"

namespace wmtk::operations {
namespace tet_mesh {
class VertexMoveToTargetPos;
}

template <>
struct OperationSettings<tet_mesh::VertexMoveToTargetPos>
{
    OperationSettings<tet_mesh::VertexAttributesUpdateBase> base_settings;
    MeshAttributeHandle<double> position;
    MeshAttributeHandle<double> move_vector;
    MeshAttributeHandle<long> todo_tag_handle;
    double alpha;
    void initialize_invariants(const TetMesh& m);
};

namespace tet_mesh {
class VertexMoveToTargetPos : public VertexAttributesUpdateBase
{
public:
    VertexMoveToTargetPos(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<VertexMoveToTargetPos>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

protected:
    bool execute() override;

protected:
    Accessor<double> m_pos_accessor;
    const OperationSettings<VertexMoveToTargetPos>& m_settings;
};

} // namespace tet_mesh
} // namespace wmtk::operations
