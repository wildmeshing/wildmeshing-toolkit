
#pragma once
#include <optional>
#include "Operation.hpp"

namespace wmtk {
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
        Mesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshCollapseEdgeOperation>& settings = {});

    std::string name() const override;


    std::vector<Tuple> modified_triangles() const;

    // return a ccw tuple from left ear if it exists, otherwise return a ccw tuple from right ear
    Tuple return_tuple() const;
    // return true if return_tuple() is from left ear, else return false and return_tuple() is from
    // right ear
    bool is_return_tuple_from_left_ear() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;
    bool before() const override;

private:
    bool m_is_output_tuple_from_left_ear;
    Tuple m_input_tuple;
    Tuple m_output_tuple;
    bool m_collapse_boundary_edges;
    bool m_collapse_boundary_vertex_to_interior;
};


} // namespace wmtk
