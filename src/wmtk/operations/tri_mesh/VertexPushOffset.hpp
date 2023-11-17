#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"
#include "VertexAttributesUpdateBase.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class VertexPushOffset;
}

template <>
struct OperationSettings<tri_mesh::VertexPushOffset>
{
    OperationSettings<tri_mesh::VertexAttributesUpdateBase> base_settings;
    MeshAttributeHandle<double> position;
    MeshAttributeHandle<long> todo_tag_handle;
    MeshAttributeHandle<long> vertex_tag_handle;
    MeshAttributeHandle<long> edge_tag_handle;
    long embedding_tag_value;
    long offset_tag_value;
    long input_tag_value;
    double offset_len;
    void initialize_invariants(const TriMesh& m);
};

namespace tri_mesh {
class VertexPushOffset : public VertexAttributesUpdateBase
{
public:
    VertexPushOffset(Mesh& m, const Tuple& t, const OperationSettings<VertexPushOffset>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

protected:
    bool execute() override;

protected:
    Accessor<double> m_pos_accessor;
    const OperationSettings<VertexPushOffset>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
