#include "TriangleAutodiffFunction.hpp"
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/AutoDiffUtils.hpp>
#include <wmtk/simplex/internal/SimplexEqualFunctor.hpp>
namespace wmtk::function {

TriangleAutodiffFunction::TriangleAutodiffFunction(
    const TriMesh& mesh,
    const MeshAttributeHandle<double>& coordinate_attribute_handle)
    : AutodiffFunction(mesh, PrimitiveType::Face, coordinate_attribute_handle)
{}

TriangleAutodiffFunction::~TriangleAutodiffFunction() = default;

std::array<TriangleAutodiffFunction::DSVec, 3> TriangleAutodiffFunction::get_variable_coordinates(
    const simplex::Simplex& domain_simplex,
    const std::optional<simplex::Simplex>& variable_simplex_opt) const
{
    std::vector<Tuple> domain_tuples;
    Tuple domain_tuple = domain_simplex.tuple();
    if (mesh().is_ccw(domain_simplex.tuple())) {
        domain_tuple = mesh().switch_vertex(domain_tuple);
    }
    Tuple starting_tuple = domain_tuple;
    // assume the ccw oriented trinagles vertices has positive signed energy

    do {
        domain_tuples.emplace_back(domain_tuple);
        domain_tuple = mesh().switch_vertex(mesh().switch_edge(domain_tuple));
    } while (domain_tuple != starting_tuple);
    assert(domain_tuples.size() == 3);

    return AutodiffFunction::get_variable_coordinates<3>(domain_tuples, variable_simplex_opt);
}

double TriangleAutodiffFunction::get_value(const simplex::Simplex& domain_simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());
    // get the pos coordinates of the triangle
    std::array<DSVec, 3> coordinates = get_variable_coordinates(domain_simplex, std::nullopt);

    // return the energy
    return eval(coordinates[0], coordinates[1], coordinates[2]).getValue();
}

Eigen::VectorXd TriangleAutodiffFunction::get_gradient(
    const simplex::Simplex& domain_simplex,
    const simplex::Simplex& variable_simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());
    // get the pos coordinates of the triangle
    std::array<DSVec, 3> coordinates =
        get_variable_coordinates(domain_simplex, std::make_optional<Simplex>(variable_simplex));

    // return the gradient
    return eval(coordinates[0], coordinates[1], coordinates[2]).getGradient();
}

Eigen::MatrixXd TriangleAutodiffFunction::get_hessian(
    const simplex::Simplex& domain_simplex,
    const simplex::Simplex& variable_simplex) const
{
    auto scope = utils::AutoDiffRAII(embedded_dimension());
    // get the pos coordinates of the triangle
    std::array<DSVec, 3> coordinates =
        get_variable_coordinates(domain_simplex, std::make_optional<Simplex>(variable_simplex));

    // return the hessian
    return eval(coordinates[0], coordinates[1], coordinates[2]).getHessian();
}


} // namespace wmtk::function