
#pragma once
#include <optional>
#include <wmtk/invariants/InvariantCollection.hpp>
#include "TupleOperation.hpp"

namespace wmtk {
class TriMeshCollapseEdgeOperation;

template <>
struct OperationSettings<TriMeshCollapseEdgeOperation>
{
    OperationSettings(const TriMesh& m);
    // are collapses between boundary and interior vertices allowed
    bool collapse_boundary_vertex_to_interior = true;
    // are collapses on boundary edges allowed
    bool collapse_boundary_edges = true;

    InvariantCollection invariants;
};

class TriMeshCollapseEdgeOperation : public TupleOperation
{
public:
    // constructor for default factory pattern construction
    TriMeshCollapseEdgeOperation(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshCollapseEdgeOperation>& settings);
    TriMeshCollapseEdgeOperation(
        TriMesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshCollapseEdgeOperation>& settings);

    std::string name() const override;


    std::vector<Tuple> modified_triangles() const;
    std::vector<Tuple> modified_primitives(PrimitiveType) const override;

    // return a ccw tuple from left ear if it exists, otherwise return a ccw tuple from right ear
    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool execute() override;

private:
    Tuple m_output_tuple;
    const OperationSettings<TriMeshCollapseEdgeOperation>& m_settings;
};


} // namespace wmtk
