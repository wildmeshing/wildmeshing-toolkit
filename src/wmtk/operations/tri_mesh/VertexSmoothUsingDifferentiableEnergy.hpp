#pragma once
#include <optional>
#include <wmtk/function/AutodiffFunction.hpp>
#include <wmtk/function/DifferentiableFunction.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/TupleOperation.hpp>
#include "TriMeshOperation.hpp"
#include "VertexSmooth.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class VertexSmoothUsingDifferentiableEnergy;
}

template <>
struct OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy>
{
    OperationSettings<tri_mesh::VertexSmooth> smooth_settings;
    std::unique_ptr<wmtk::function::DifferentiableFunction> energy;
    bool second_order = true;
    bool line_search = false;
    void initialize_invariants(const TriMesh& m);
    double step_size = 1.0;
};

namespace tri_mesh {
class VertexSmoothUsingDifferentiableEnergy : public TriMeshOperation, protected TupleOperation
{
public:
    VertexSmoothUsingDifferentiableEnergy(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings);

    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }

protected:
    bool before() const override;
    bool execute() override;

protected:
    Accessor<double> m_uv_pos_accessor;
    const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
