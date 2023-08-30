#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include "TupleOperation.hpp"
#include "TriMeshCollapseEdgeOperation.hpp"

namespace wmtk {
class TriMeshCollapseEdgeToMidpointOperation;

template <>
struct OperationSettings<TriMeshCollapseEdgeToMidpointOperation>
{
    OperationSettings<TriMeshCollapseEdgeOperation> collapse_settings;
    // handle to vertex position
    MeshAttributeHandle<double> position;
    // too long edges get ignored
    double max_squared_length = std::numeric_limits<double>::max();
    // in case of a collapse between an interior and a boundary vertex, the vertex is not moved to
    // the midpoint but to the boundary vertex position
    bool collapse_towards_boundary = false;
    void initialize_invariants(const TriMesh& m);

    // debug functionality to make sure operations are constructed properly
    bool are_invariants_initialized() const;
};

class TriMeshCollapseEdgeToMidpointOperation : public TupleOperation
{
public:
    TriMeshCollapseEdgeToMidpointOperation(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshCollapseEdgeToMidpointOperation>& settings);

    std::string name() const override;

    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }
    std::vector<Tuple> modified_primitives(PrimitiveType) const override;

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_output_tuple;

    Accessor<double> m_pos_accessor;
    const OperationSettings<TriMeshCollapseEdgeToMidpointOperation>& m_settings;

};


} // namespace wmtk
