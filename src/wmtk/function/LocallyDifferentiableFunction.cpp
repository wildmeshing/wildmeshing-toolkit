#include "LocallyDifferentiableFunction.hpp"
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/upper_level_cofaces.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
namespace wmtk {
namespace function {
LocallyDifferentiableFunction::LocallyDifferentiableFunction(
    const Mesh& mesh,
    const std::unique_ptr<DifferentiablePerSimplexFunction>& function)
    : Function(mesh)
    , m_function(std::move(function))
{}

LocallyDifferentiableFunction::~LocallyDifferentiableFunction() = default;

long LocallyDifferentiableFunction::embedded_dimension() const
{
    return mesh().get_attribute_dimension(m_function->get_variable_attribute_handle());
}


Eigen::VectorXd LocallyDifferentiableFunction::get_one_ring_gradient(
    const Simplex& my_simplex,
    const PrimitiveType& cofaces_type) const
{
    m_function->assert_function_type(my_simplex.primitive_type());
    std::vector<Tuple> coface_tuples =
        simplex::upper_level_cofaces_tuples(mesh(), my_simplex, cofaces_type);

    return get_gradient_sum(
        simplex::utils::tuple_vector_to_homogeneous_simplex_vector(coface_tuples, cofaces_type));
}
Eigen::MatrixXd LocallyDifferentiableFunction::get_one_ring_hessian(
    const Simplex& my_simplex,
    const PrimitiveType& cofaces_type) const
{
    m_function->assert_function_type(my_simplex.primitive_type());
    std::vector<Tuple> coface_tuples =
        simplex::upper_level_cofaces_tuples(mesh(), my_simplex, cofaces_type);

    return get_hessian_sum(
        simplex::utils::tuple_vector_to_homogeneous_simplex_vector(coface_tuples, cofaces_type));
}

double LocallyDifferentiableFunction::get_value_sum(
    const std::vector<Simplex>& coface_simplices) const
{
    double v = 0;
    for (const Simplex& cell : coface_simplices) {
        v += m_function->get_value(cell);
    }
    return v;
}


Eigen::VectorXd LocallyDifferentiableFunction::get_gradient_sum(
    const std::vector<Simplex>& coface_simplices) const
{
    Eigen::VectorXd g = Eigen::VectorXd::Zero(embedded_dimension());
    for (const Simplex& cell : coface_simplices) {
        g += m_function->get_gradient(cell);
    }
    return g;
}
Eigen::MatrixXd LocallyDifferentiableFunction::get_hessian_sum(
    const std::vector<Simplex>& coface_simplices) const
{
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(embedded_dimension(), embedded_dimension());
    for (const Simplex& cell : coface_simplices) {
        h += m_function->get_hessian(cell);
    }
    return h;
}

} // namespace function

} // namespace wmtk