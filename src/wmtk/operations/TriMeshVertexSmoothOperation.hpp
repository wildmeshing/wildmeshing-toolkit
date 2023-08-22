#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include "Operation.hpp"

namespace wmtk::operations {
class TriMeshVertexSmoothOperation;

template <>
struct OperationSettings<TriMeshVertexSmoothOperation>
{
    MeshAttributeHandle<double> position;
    bool smooth_boundary = false;
};

class TriMeshVertexSmoothOperation : public Operation
{
public:
    TriMeshVertexSmoothOperation(
        wmtk::Mesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshVertexSmoothOperation>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_tuple;
    Accessor<double> m_pos_accessor;
    const OperationSettings<TriMeshVertexSmoothOperation>& m_settings;
};


} // namespace wmtk::operations
