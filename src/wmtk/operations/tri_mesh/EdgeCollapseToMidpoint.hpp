#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include <wmtk/operations/tri_mesh/TriMeshOperation.hpp>
#include "EdgeCollapse.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeCollapseToMidpoint;
}

template <>
struct OperationSettings<tri_mesh::EdgeCollapseToMidpoint>
{
    OperationSettings<tri_mesh::EdgeCollapse> collapse_settings;
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

namespace tri_mesh {
class EdgeCollapseToMidpoint : public TriMeshOperation, private TupleOperation
{
public:
    EdgeCollapseToMidpoint(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<EdgeCollapseToMidpoint>& settings);

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
    const OperationSettings<EdgeCollapseToMidpoint>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
