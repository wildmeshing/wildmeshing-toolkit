#pragma once
#include <wmtk/Primitive.hpp>
#include <wmtk/function/Function.hpp>
#include <wmtk/operations/Operation.hpp>
#include "VertexAttributesUpdateBase.hpp"

namespace wmtk {
namespace function {
class DifferentiableFunction;

}
} // namespace wmtk
namespace wmtk::operations {
namespace tri_mesh {
class VertexSmoothing;
}

template <>
struct OperationSettings<tri_mesh::VertexSmoothing>
{
    OperationSettings<tri_mesh::VertexAttributesUpdateBase> base_settings;
    std::unique_ptr<wmtk::function::Function> energy;
    // coordinate for teh attribute used to evaluate the energy
    MeshAttributeHandle<double> coordinate_handle;
    bool smooth_boundary = false;

    bool second_order = true;
    bool line_search = false;
    void initialize_invariants(const TriMesh& m);
    double step_size = 1.0;
};

namespace tri_mesh {
class VertexSmoothing : public VertexAttributesUpdateBase
{
protected:
    VertexSmoothing(Mesh& m, const Tuple& t, const OperationSettings<VertexSmoothing>& settings);

public:
    std::string name() const override;

protected:
    MeshAttributeHandle<double> coordinate_handle() const { return m_settings.coordinate_handle; }

    Accessor<double> coordinate_accessor();
    ConstAccessor<double> const_coordinate_accessor() const;
    const OperationSettings<VertexSmoothing>& m_settings;
};

} // namespace tri_mesh
} // namespace wmtk::operations
