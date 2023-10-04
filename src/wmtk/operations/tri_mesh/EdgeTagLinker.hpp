#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeTagLinker;
}

template <>
struct OperationSettings<tri_mesh::EdgeTagLinker>
{
    MeshAttributeHandle<long> vertex_tag;
    MeshAttributeHandle<long> edge_tag;
    long input_tag_value;
    long embedding_tag_value;
    long offset_tag_value;
    InvariantCollection invariants;
};

namespace tri_mesh {
class EdgeTagLinker : public TriMeshOperation, private TupleOperation
{
public:
    EdgeTagLinker(Mesh& m, const Tuple& t, const OperationSettings<EdgeTagLinker>& settings);

    std::string name() const override;

    Tuple return_tuple() const;
    std::vector<Tuple> modified_primitives(PrimitiveType) const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

    using TriMeshOperation::hash_accessor;

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;
    Accessor<long> m_vertex_tag_accessor;
    Accessor<long> m_edge_tag_accessor;
    const OperationSettings<EdgeTagLinker>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
