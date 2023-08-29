
#pragma once
#include <optional>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include "TupleOperation.hpp"

namespace wmtk {
class TriMeshCollapseEdgeOperation;

template <>
struct OperationSettings<TriMeshCollapseEdgeOperation>
{
    OperationSettings();
    // are collapses between boundary and interior vertices allowed
    bool collapse_boundary_vertex_to_interior = true;
    // are collapses on boundary edges allowed
    bool collapse_boundary_edges = true;

    InvariantCollection invariants;

    void initialize_invariants(const TriMesh& m);

    // debug functionality to make sure operations are constructed properly
    bool are_invariants_initialized() const;
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
    bool before() const override;

private:
    Tuple m_output_tuple;
    const OperationSettings<TriMeshCollapseEdgeOperation>& m_settings;
};


} // namespace wmtk
