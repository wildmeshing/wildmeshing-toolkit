#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/function/Function.hpp>
#include <wmtk/operations/Operation.hpp>
#include "VertexAttributesUpdateBase.hpp"

namespace wmtk {
namespace function {
class DifferentiableFunction;

}
namespace operations {
namespace tri_mesh {
class VertexSmoothing;
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

    bool second_order = true;
    bool line_search = false;
    double step_size = 1.0;

    void create_invariants();
};

namespace tri_mesh {
class VertexSmoothing : public VertexAttributesUpdateBase
{
protected:
    VertexSmoothing(
        Mesh& m,
        const Simplex& t,
        const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings);

public:
    std::string name() const override;
    bool execute() override;

protected:
    MeshAttributeHandle<double> coordinate_handle() const { return m_settings.coordinate_handle; }

    Accessor<double> coordinate_accessor();
    ConstAccessor<double> const_coordinate_accessor() const;
    const OperationSettings<VertexSmoothing>& m_settings;
};

} // namespace tri_mesh
} // namespace operations
} // namespace wmtk
// provides overload for factory
