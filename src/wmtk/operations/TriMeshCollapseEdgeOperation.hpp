
#pragma once
#include <optional>
#include "Operation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class TriMeshEdgeCollapse;
}

template <>
struct OperationSettings<tri_mesh::TriMeshEdgeCollapse>
{
    // are collapses between boundary and interior vertices allowed
    bool collapse_boundary_vertex_to_interior = true;
    // are collapses on boundary edges allowed
    bool collapse_boundary_edges = true;
};

namespace tri_mesh {
class TriMeshEdgeCollapse : public Operation
{
public:
    TriMeshEdgeCollapse(
        wmtk::Mesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshEdgeCollapse>& settings = {});

    std::string name() const override;


    std::vector<Tuple> modified_triangles() const;

    // return a ccw tuple from left ear if it exists, otherwise return a ccw tuple from right ear
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;
    bool before() const override;

private:
    Tuple m_input_tuple;
    Tuple m_output_tuple;
    const OperationSettings<TriMeshEdgeCollapse>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
