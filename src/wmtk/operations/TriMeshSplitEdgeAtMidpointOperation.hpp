#pragma once
#include <wmtk/TriMesh.hpp>
#include "Operation.hpp"

namespace wmtk {
class TriMeshSplitEdgeAtMidpointOperation;

template <>
struct OperationSettings<TriMeshSplitEdgeAtMidpointOperation>
{
    MeshAttributeHandle<double> position;
    double min_squared_length = -1;
};

class TriMeshSplitEdgeAtMidpointOperation : public Operation
{
public:
    TriMeshSplitEdgeAtMidpointOperation(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<TriMeshSplitEdgeAtMidpointOperation>& settings);

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
    double m_min_squared_length;

    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
};


} // namespace wmtk
