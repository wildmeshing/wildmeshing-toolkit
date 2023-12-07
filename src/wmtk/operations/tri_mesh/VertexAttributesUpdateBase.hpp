#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>

#include "TriMeshOperation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class VertexAttributesUpdateBase;
}

template <>
struct OperationSettings<tri_mesh::VertexAttributesUpdateBase> : public OperationSettingsBase
{
    OperationSettings<tri_mesh::VertexAttributesUpdateBase>(TriMesh& m)
        : m_mesh(m)
    {}

    TriMesh& m_mesh;

    void create_invariants();
};

namespace tri_mesh {
class VertexAttributesUpdateBase : public TriMeshOperation, protected TupleOperation
{
public:
    VertexAttributesUpdateBase(
        Mesh& m,
        const Simplex& t,
        const OperationSettings<VertexAttributesUpdateBase>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

    const Tuple& return_tuple() const;
    std::vector<Tuple> modified_primitives(PrimitiveType) const override;

protected:
    bool execute() override;

protected:
    Tuple m_output_tuple;
    // const OperationSettings<VertexAttributesUpdateBase>& m_settings;// TODO unused variable
};

} // namespace tri_mesh
} // namespace wmtk::operations
