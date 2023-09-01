#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include "Operation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class VertexSmooth;
}

template <>
struct OperationSettings<tri_mesh::VertexSmooth>
{
    MeshAttributeHandle<double> position;
    bool smooth_boundary = false;
};

namespace tri_mesh {
class VertexSmooth : public Operation
{
public:
    VertexSmooth(Mesh& m, const Tuple& t, const OperationSettings<VertexSmooth>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

    const Tuple& return_tuple() const;

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_input_tuple;
    Tuple m_output_tuple;
    Accessor<double> m_pos_accessor;
    const OperationSettings<VertexSmooth>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
