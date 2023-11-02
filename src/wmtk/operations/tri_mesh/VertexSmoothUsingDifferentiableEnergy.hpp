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
} // namespace wmtk
namespace wmtk::operations {
namespace tri_mesh {
class VertexSmoothUsingDifferentiableEnergy;
}

template <>
struct OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy>
{
    OperationSettings<tri_mesh::VertexAttributesUpdateBase> base_settings;
    std::unique_ptr<wmtk::function::LocalDifferentiableFunction> energy;
    // coordinate for teh attribute used to evaluate the energy
    MeshAttributeHandle<double> coordinate_handle;
    bool smooth_boundary = false;

    bool second_order = true;
    bool line_search = false;
    void initialize_invariants(const TriMesh& m);
    double step_size = 1.0;
};

namespace tri_mesh {
class VertexSmoothUsingDifferentiableEnergy : public VertexAttributesUpdateBase
{
protected:
    VertexSmoothUsingDifferentiableEnergy(
        Mesh& m,
        const Tuple& t,
        const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings);

public:
    std::string name() const override;

    static PrimitiveType primitive_type() { return PrimitiveType::Vertex; }


protected:
    function::utils::DifferentiableFunctionEvaluator get_function_evaluator(
        Accessor<double>& accessor) const;
    MeshAttributeHandle<double> coordinate_handle() const { return m_settings.coordinate_handle; }

    Accessor<double> coordinate_accessor();
    ConstAccessor<double> const_coordinate_accessor() const;
    const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
// provides overload for factory
#include <wmtk/operations/tri_mesh/internal/VertexSmoothUsingDifferentiableEnergyFactory.hpp>
