#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/function/LocalDifferentiableFunction.hpp>
#include <wmtk/function/utils/DifferentiableFunctionEvaluator.hpp>
#include <wmtk/operations/Operation.hpp>
#include "VertexAttributesUpdateBase.hpp"

namespace wmtk {
namespace function {
class DifferentiableFunction;

}
namespace operations {
namespace tri_mesh {
class VertexSmoothUsingDifferentiableEnergy;
}

template <>
struct OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy>
    : public OperationSettings<tri_mesh::VertexAttributesUpdateBase>
{
    OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy>(TriMesh& m)
        : OperationSettings<tri_mesh::VertexAttributesUpdateBase>(m)
    {}

    std::unique_ptr<wmtk::function::LocalDifferentiableFunction> energy;
    // coordinate for teh attribute used to evaluate the energy
    MeshAttributeHandle<double> coordinate_handle;
    bool smooth_boundary = false;
    std::shared_ptr<TriMesh> uv_mesh_ptr;
    std::optional<MeshAttributeHandle<double>> uv_handle;
    bool second_order = true;
    bool line_search = false;
    double step_size = 1.0;
    bool do_seamless_optimization = false;
    void create_invariants();
};

namespace tri_mesh {
class VertexSmoothUsingDifferentiableEnergy : public VertexAttributesUpdateBase
{
protected:
    VertexSmoothUsingDifferentiableEnergy(
        Mesh& m,
        const Simplex& t,
        const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings);

public:
    std::string name() const override;

protected:
    function::utils::DifferentiableFunctionEvaluator get_function_evaluator(
        Accessor<double>& accessor) const;
    MeshAttributeHandle<double> coordinate_handle() const { return m_settings.coordinate_handle; }

    Accessor<double> coordinate_accessor();
    ConstAccessor<double> const_coordinate_accessor() const;
    const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& m_settings;
};

} // namespace tri_mesh
} // namespace operations
} // namespace wmtk
// provides overload for factory
#include <wmtk/operations/tri_mesh/internal/VertexSmoothUsingDifferentiableEnergyFactory.hpp>
