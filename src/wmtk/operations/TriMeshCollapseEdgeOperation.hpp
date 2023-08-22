
#pragma once
#include <optional>
#include "Operation.hpp"

namespace wmtk::operations {
class TriMeshCollapseEdgeOperation;

template <>
struct OperationSettings<TriMeshCollapseEdgeOperation>
{
    // are collapses between boundary and interior vertices allowed
    bool collapse_boundary_vertex_to_interior = true;
    // are collapses on boundary edges allowed
    bool collapse_boundary_edges = true;
};

class TriMeshCollapseEdgeOperation : public Operation
{
public:
    TriMeshCollapseEdgeOperation(
        wmtk::Mesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshCollapseEdgeOperation>& settings = {});

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
    const OperationSettings<TriMeshCollapseEdgeOperation>& m_settings;
};


} // namespace wmtk::operations
