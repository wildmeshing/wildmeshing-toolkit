#include "AutodiffFunction.hpp"
#include <wmtk/function/utils/AutoDiffRAII.hpp>

namespace wmtk::function {

AutodiffFunction::AutodiffFunction(
    const Mesh& mesh,
    const PrimitiveType& simplex_type,
    const MeshAttributeHandle<double>& variable_attribute_handle)
    : PerSimplexDifferentiableFunction(mesh, simplex_type, variable_attribute_handle)
{}

AutodiffFunction::~AutodiffFunction() = default;

auto AutodiffFunction::get_value_autodiff(const Simplex& simplex) const -> DScalar 
{
    assert(simplex.primitive_type() == get_simplex_type());
    return get_value_autodiff(simplex.tuple());
}
double AutodiffFunction::get_value(const Tuple& simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());
    auto v = get_value_autodiff(simplex);
    return v.getValue();
}

Eigen::VectorXd AutodiffFunction::get_gradient(const Tuple& simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());
    auto v = get_value_autodiff(simplex);
    return v.getGradient();
}
Eigen::MatrixXd AutodiffFunction::get_hessian(const Tuple& simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());
    auto v = get_value_autodiff(simplex);
    return v.getHessian();
}
} // namespace wmtk::function
