#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include "Operation.hpp"

namespace wmtk {
class TriMeshVertexTangentialSmoothOperation;

template <>
struct OperationSettings<TriMeshVertexTangentialSmoothOperation>
{
    MeshAttributeHandle<double> position;
    bool smooth_boundary = false;
    double damping_factor = 1.0;
};

class TriMeshVertexTangentialSmoothOperation : public Operation
{
public:
    TriMeshVertexTangentialSmoothOperation(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshVertexTangentialSmoothOperation>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_tuple;
    Accessor<double> m_pos_accessor;
    const OperationSettings<TriMeshVertexTangentialSmoothOperation>& m_settings;
};


} // namespace wmtk
