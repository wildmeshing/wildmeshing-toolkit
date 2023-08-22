#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include "../Operation.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class EdgeCollapseToMidpoint;
}

template <>
struct OperationSettings<tri_mesh::EdgeCollapseToMidpoint>
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

namespace tri_mesh {
class EdgeCollapseToMidpoint : public Operation
{
public:
    EdgeCollapseToMidpoint(
        wmtk::Mesh& m,
        const Tuple& t,
        const OperationSettings<EdgeCollapseToMidpoint>& settings);

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
    const OperationSettings<EdgeCollapseToMidpoint>& m_settings;

    Eigen::Vector3d p0;
    Eigen::Vector3d p1;
};

} // namespace tri_mesh
} // namespace wmtk::operations
