#pragma once
#include <optional>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include <wmtk/operations/tri_mesh/TriMeshOperation.hpp>
#include "EdgeCollapse.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class ExtremeOptCollapse;
}

template <>
struct OperationSettings<tri_mesh::ExtremeOptCollapse>
    : public OperationSettings<tri_mesh::EdgeCollapse>
{
    OperationSettings<tri_mesh::ExtremeOptCollapse>(TriMesh& m)
        : OperationSettings<tri_mesh::EdgeCollapse>(m)
    {}

    // handle to vertex position
    MeshAttributeHandle<double> position;
    // too long edges get ignored
    double max_squared_length = std::numeric_limits<double>::max();
    // in case of a collapse between an interior and a boundary vertex, the vertex is not moved to
    // the midpoint but to the boundary vertex position
    bool collapse_towards_boundary = false;

    std::shared_ptr<TriMesh> uv_mesh_ptr;
    MeshAttributeHandle<double> uv_handle;

    void create_invariants();
};

namespace tri_mesh {
class ExtremeOptCollapse : public EdgeCollapse
{
public:
    ExtremeOptCollapse(
        Mesh& m,
        const Simplex& t,
        const OperationSettings<ExtremeOptCollapse>& settings);

    std::string name() const override;
    std::vector<double> priority() const override;
    static PrimitiveType primitive_type() { return PrimitiveType::Edge; }

protected:
    bool before() const override;
    bool execute() override;

private:
    Accessor<double> m_pos_accessor;
    Accessor<double> m_uv_accessor;

    const OperationSettings<ExtremeOptCollapse>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
