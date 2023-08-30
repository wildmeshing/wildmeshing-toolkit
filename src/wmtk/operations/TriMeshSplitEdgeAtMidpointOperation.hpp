#pragma once
#include <wmtk/TriMesh.hpp>
#include "TupleOperation.hpp"
#include "TriMeshSplitEdgeOperation.hpp"

namespace wmtk {
class TriMeshSplitEdgeAtMidpointOperation;

template <>
struct OperationSettings<TriMeshSplitEdgeAtMidpointOperation>
{
    OperationSettings<TriMeshSplitEdgeOperation> split_settings;
    // handle to vertex position
    MeshAttributeHandle<double> position;
    // too short edges get ignored
    double min_squared_length = -1;

    void initialize_invariants(const TriMesh& m);

    // debug functionality to make sure operations are constructed properly
    bool are_invariants_initialized() const;
};

class TriMeshSplitEdgeAtMidpointOperation : public TupleOperation
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
    Tuple m_output_tuple;
    Accessor<double> m_pos_accessor;

    const OperationSettings<TriMeshSplitEdgeAtMidpointOperation>& m_settings;

    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
};


} // namespace wmtk
