#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include "Operation.hpp"

namespace wmtk {
class TriMeshCollapseEdgeToMidpointOperation;

template <>
struct OperationSettings<TriMeshCollapseEdgeToMidpointOperation>
{
    // handle to vertex position
    MeshAttributeHandle<double> position;
    // too long edges get ignored
    double max_squared_length = std::numeric_limits<double>::max();
    // collapse on boundary
    bool collapse_boundary_edges = true;
    // in case of a collapse between an interior and a boundary vertex, the vertex is not moved to
    // the midpoint but to the boundary vertex position
    bool collapse_towards_boundary = false;
};

class TriMeshCollapseEdgeToMidpointOperation : public Operation
{
public:
    TriMeshCollapseEdgeToMidpointOperation(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshCollapseEdgeToMidpointOperation>& settings);

    std::string name() const override;

    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_input_tuple;
    Tuple m_output_tuple;

    Accessor<double> m_pos_accessor;
    const OperationSettings<TriMeshCollapseEdgeToMidpointOperation>& m_settings;

    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
};


} // namespace wmtk
