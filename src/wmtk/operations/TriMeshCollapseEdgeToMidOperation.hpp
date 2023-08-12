#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include "Operation.hpp"

namespace wmtk {
class TriMeshCollapseEdgeToMidOperation;

template <>
struct OperationSettings<TriMeshCollapseEdgeToMidOperation>
{
    MeshAttributeHandle<double> position;
    double max_squared_length = std::numeric_limits<double>::max();
    bool collapse_boundary_edges = true;
    bool collapse_towards_boundary = false;
};

class TriMeshCollapseEdgeToMidOperation : public Operation
{
public:
    TriMeshCollapseEdgeToMidOperation(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshCollapseEdgeToMidOperation>& settings);

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
    double m_max_squared_length;
    bool m_collapse_boundary_edges;
    bool m_collapse_towards_boundary;

    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
};


} // namespace wmtk
