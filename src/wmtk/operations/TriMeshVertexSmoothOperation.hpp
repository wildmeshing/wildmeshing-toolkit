#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include "Operation.hpp"

namespace wmtk {
class TriMeshVertexSmoothOperation : public Operation
{
public:
    struct Settings
    {
        MeshAttributeHandle<double> position;
        bool smooth_boundary = false;
    };

    TriMeshVertexSmoothOperation(Mesh& m, const Tuple& t, const Settings& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_tuple;
    Accessor<double> m_pos_accessor;
};


} // namespace wmtk