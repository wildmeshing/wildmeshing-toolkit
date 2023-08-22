#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include "Operation.hpp"

namespace wmtk::operations {
class TriMeshVertexSmooth;

template <>
struct OperationSettings<TriMeshVertexSmooth>
{
    MeshAttributeHandle<double> position;
    bool smooth_boundary = false;
};

class TriMeshVertexSmooth : public Operation
{
public:
    TriMeshVertexSmooth(
        wmtk::Mesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshVertexSmooth>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_tuple;
    Accessor<double> m_pos_accessor;
    const OperationSettings<TriMeshVertexSmooth>& m_settings;
};


} // namespace wmtk::operations
