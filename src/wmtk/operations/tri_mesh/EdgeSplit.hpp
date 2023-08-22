#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include "../Operation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeSplit;
}

template <>
struct OperationSettings<tri_mesh::EdgeSplit>
{
    bool split_boundary_edges = true;
};

namespace tri_mesh {
class EdgeSplit : public Operation
{
public:
    EdgeSplit(wmtk::Mesh& m, const Tuple& t, const OperationSettings<EdgeSplit>& settings);

    std::string name() const override;

    Tuple new_vertex() const;
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;
    bool before() const override;

private:
    Tuple m_input_tuple;
    Tuple m_output_tuple;

    const OperationSettings<EdgeSplit>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
