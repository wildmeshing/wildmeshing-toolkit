#include "AutodiffFunction.hpp"
#include <wmtk/function/utils/AutoDiffRAII.hpp>

namespace wmtk::function {

AutodiffFunction::AutodiffFunction(
    const Mesh& mesh,
    const PrimitiveType& simplex_type,
    const MeshAttributeHandle<double>& variable_attribute_handle)
    : DifferentiablePerSimplexFunction(mesh, simplex_type, variable_attribute_handle)
{}

AutodiffFunction::~AutodiffFunction() = default;
double AutodiffFunction::get_value(const Simplex& simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());
    return get_value_autodiff(simplex).getValue();
}
Eigen::VectorXd AutodiffFunction::get_gradient(const Simplex& simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());
    auto v = get_value_autodiff(simplex);
    return v.getGradient();
}
Eigen::MatrixXd AutodiffFunction::get_hessian(const Simplex& simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());
    return get_value_autodiff(simplex).getHessian();
}
} // namespace wmtk::function
