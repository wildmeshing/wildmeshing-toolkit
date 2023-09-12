#include "AutodiffFunction.hpp"

using namespace wmtk;
using namespace wmtk::function;

AutodiffFunction::AutodiffFunction(
    const Mesh& mesh,
    const MeshAttributeHandle<double>& vertex_attribute_handle)
    : DifferentiableFunction(mesh, vertex_attribute_handle){};

double AutodiffFunction::get_value(const Tuple& tuple) const
{
    return get_value_autodiff(tuple).getValue();
}
Eigen::VectorXd AutodiffFunction::get_gradient(const Tuple& tuple) const
{
    return get_value_autodiff(tuple).getGradient();
}
Eigen::MatrixXd AutodiffFunction::get_hessian(const Tuple& tuple) const
{
    return get_value_autodiff(tuple).getHessian();
}