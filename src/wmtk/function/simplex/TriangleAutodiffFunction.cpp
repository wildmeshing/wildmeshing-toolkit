#include "TriangleAutodiffFunction.hpp"
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/AutoDiffUtils.hpp>
#include <wmtk/simplex/internal/SimplexEqualFunctor.hpp>
namespace wmtk::function {

TriangleAutodiffFunction::TriangleAutodiffFunction(
    const TriMesh& mesh,
    const MeshAttributeHandle<double>& coordinate_attribute_handle)
    : PerSimplexDifferentiableAutodiffFunction(
          mesh,
          PrimitiveType::Face,
          coordinate_attribute_handle)
{
    const PrimitiveType pt = get_coordinate_attribute_primitive_type();
    if (pt == PrimitiveType::Face) {
        throw std::runtime_error(
            "TriangleAutodiffFunction does not work on functions defined on faces");
    }
}

TriangleAutodiffFunction::~TriangleAutodiffFunction() = default;

std::array<TriangleAutodiffFunction::DSVec, 3> TriangleAutodiffFunction::get_coordinates(
    const simplex::Simplex& domain_simplex,
    const std::optional<simplex::Simplex>& variable_simplex_opt) const
{
    return PerSimplexDifferentiableAutodiffFunction::get_coordinates_T<3>(
        domain_simplex,
        variable_simplex_opt);
}

double TriangleAutodiffFunction::get_value(const simplex::Simplex& domain_simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());
    // get the pos coordinates of the triangle
    std::array<DSVec, 3> coordinates = get_coordinates(domain_simplex);

    // return the energy
    return eval(domain_simplex, coordinates).getValue();
}

Eigen::VectorXd TriangleAutodiffFunction::get_gradient(
    const simplex::Simplex& domain_simplex,
    const simplex::Simplex& variable_simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());
    // get the pos coordinates of the triangle
    std::array<DSVec, 3> coordinates = get_coordinates(domain_simplex, variable_simplex);

    // return the gradient
    return eval(domain_simplex, coordinates).getGradient();
}

Eigen::MatrixXd TriangleAutodiffFunction::get_hessian(
    const simplex::Simplex& domain_simplex,
    const simplex::Simplex& variable_simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());
    // get the pos coordinates of the triangle
    std::array<DSVec, 3> coordinates = get_coordinates(domain_simplex, variable_simplex);

    // return the hessian
    return eval(domain_simplex, coordinates).getHessian();
}


} // namespace wmtk::function
