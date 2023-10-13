#include "AutodiffFunction.hpp"
#include <wmtk/function/utils/AutoDiffRAII.hpp>

namespace wmtk::function {

AutodiffFunction::AutodiffFunction(
    const Mesh& mesh,
    const MeshAttributeHandle<double>& vertex_attribute_handle)
    : DifferentiableFunction(mesh, vertex_attribute_handle)
{}

AutodiffFunction::~AutodiffFunction() = default;
double AutodiffFunction::get_value(const Tuple& tuple) const
{
    auto scope = AutoDiffRAII(embedded_dimension());
    return get_value_autodiff(tuple).getValue();
}
Eigen::VectorXd AutodiffFunction::get_gradient(const Tuple& tuple) const
{
    auto scope = AutoDiffRAII(embedded_dimension());
    return get_value_autodiff(tuple).getGradient();
}
Eigen::MatrixXd AutodiffFunction::get_hessian(const Tuple& tuple) const
{
    auto scope = AutoDiffRAII(embedded_dimension());
    return get_value_autodiff(tuple).getHessian();
}
} // namespace wmtk::function
