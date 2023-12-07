#pragma once
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "EdgeSplit.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeSplitAtMidpoint;
}

template <>
struct OperationSettings<tri_mesh::EdgeSplitAtMidpoint> : public OperationSettingsBase
{
    // constructor
    OperationSettings<tri_mesh::EdgeSplitAtMidpoint>(TriMesh& m)
        : m_mesh(m)
        , split_settings(m)
    {}

    TriMesh& m_mesh;

    OperationSettings<tri_mesh::EdgeSplit> split_settings;
    // handle to vertex position
    MeshAttributeHandle<double> position;
    // too short edges get ignored
    double min_squared_length = -1;

    void create_invariants();
};

namespace tri_mesh {
class EdgeSplitAtMidpoint : public EdgeSplit
{
public:
    EdgeSplitAtMidpoint(
        Mesh& m,
        const Simplex& t,
        const OperationSettings<EdgeSplitAtMidpoint>& settings);

    std::string name() const override;

    Tuple return_tuple() const;

    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool before() const override;
    bool execute() override;

private:
    Tuple m_output_tuple;
    Accessor<double> m_pos_accessor;

    const OperationSettings<EdgeSplitAtMidpoint>& m_settings;

    Eigen::VectorXd coord0;
    Eigen::VectorXd coord1;
};

} // namespace tri_mesh
} // namespace wmtk::operations
