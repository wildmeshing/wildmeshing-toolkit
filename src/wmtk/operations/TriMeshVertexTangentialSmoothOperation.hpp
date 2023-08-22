#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include "Operation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class TriMeshVertexTangentialSmooth;
}

template <>
struct OperationSettings<tri_mesh::TriMeshVertexTangentialSmooth>
{
    MeshAttributeHandle<double> position;
    bool smooth_boundary = false;
    double damping_factor = 1.0;
};

namespace tri_mesh {
class TriMeshVertexTangentialSmooth : public Operation
{
public:
    TriMeshVertexTangentialSmooth(
        wmtk::Mesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshVertexTangentialSmooth>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_tuple;
    Accessor<double> m_pos_accessor;
    const OperationSettings<TriMeshVertexTangentialSmooth>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
