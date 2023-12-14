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
    : public OperationSettings<tri_mesh::EdgeCollapse>
{
    OperationSettings<tri_mesh::EdgeCollapseToMidpoint>(TriMesh& m)
        : OperationSettings<tri_mesh::EdgeCollapse>(m)
    {}


    // handle to vertex position
    MeshAttributeHandle<double> position;
    // too long edges get ignored
    double max_squared_length = std::numeric_limits<double>::max();
    // in case of a collapse between an interior and a boundary vertex, the vertex is not moved to
    // the midpoint but to the boundary vertex position
    bool collapse_towards_boundary = false;
    void create_invariants();
};

namespace tri_mesh {
class EdgeCollapseToMidpoint : public EdgeCollapse
{
public:
    EdgeCollapseToMidpoint(
        Mesh& m,
        const Simplex& t,
        const OperationSettings<EdgeCollapseToMidpoint>& settings);

    std::string name() const override;


    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool before() const override;
    bool execute() override;

private:
    Accessor<double> m_pos_accessor;
    const OperationSettings<EdgeCollapseToMidpoint>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
