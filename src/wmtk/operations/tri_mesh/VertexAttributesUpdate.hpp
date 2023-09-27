#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class VertexAttributesUpdate;
}

template <>
struct OperationSettings<tri_mesh::VertexAttributesUpdate>
{
    InvariantCollection invariants;
    void initialize_invariants(const TriMesh& m);
};

namespace tri_mesh {
class VertexAttributesUpdate : public TriMeshOperation, protected TupleOperation
{
public:
    VertexAttributesUpdate(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<VertexAttributesUpdate>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

    const Tuple& return_tuple() const;

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_output_tuple;
    const OperationSettings<VertexAttributesUpdate>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
