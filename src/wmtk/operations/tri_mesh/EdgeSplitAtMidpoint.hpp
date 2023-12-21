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
struct OperationSettings<tri_mesh::EdgeSplitAtMidpoint> : public OperationSettings<tri_mesh::EdgeSplit>
{
    // constructor
    OperationSettings<tri_mesh::EdgeSplitAtMidpoint>(TriMesh& m)
        : OperationSettings<tri_mesh::EdgeSplit>(m)
    {}


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


    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool before() const override;
    bool execute() override;

private:
    Accessor<double> m_pos_accessor;

    const OperationSettings<EdgeSplitAtMidpoint>& m_settings;

};

} // namespace tri_mesh
} // namespace wmtk::operations
