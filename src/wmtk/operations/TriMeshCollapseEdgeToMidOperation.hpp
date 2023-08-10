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

    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
};


} // namespace wmtk
