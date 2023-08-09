#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include "Operation.hpp"

namespace wmtk {
class TriMeshVertexSmoothOperation : public Operation
{
public:
    struct Handles
    {
        MeshAttributeHandle<double> position;
    };

    TriMeshVertexSmoothOperation(Mesh& m, const Tuple& t, const Handles& handles);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

protected:
    bool execute() override;
    bool before() const override;

private:
    Tuple m_tuple;
    Accessor<double> m_pos_accessor;
};


} // namespace wmtk
