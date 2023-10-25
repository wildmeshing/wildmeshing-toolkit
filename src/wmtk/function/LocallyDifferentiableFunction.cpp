#include "LocallyDifferentiableFunction.hpp"
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/upper_level_cofaces.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
namespace wmtk {
namespace function {
LocallyDifferentiableFunction::LocallyDifferentiableFunction(
    std::shared_ptr<DifferentiablePerSimplexFunction>&& function,
    const PrimitiveType& my_simplex_type)
    : Function(function)
    , m_my_simplex_type(my_simplex_type)
{
    m_function->assert_function_type(m_my_simplex_type);
}

LocallyDifferentiableFunction::~LocallyDifferentiableFunction() = default;

Eigen::VectorXd LocallyDifferentiableFunction::get_local_gradient(const Tuple& my_tuple) const
{
    return get_local_gradient(Simplex(m_my_simplex_type, my_tuple));
}

Eigen::MatrixXd LocallyDifferentiableFunction::get_local_hessian(const Tuple& my_tuple) const
{
    return get_local_hessian(Simplex(m_my_simplex_type, my_tuple));
}

Eigen::VectorXd LocallyDifferentiableFunction::get_local_gradient(const Simplex& my_simplex) const
{
    std::vector<Tuple> coface_tuples = simplex::upper_level_cofaces_tuples(
        m_function->mesh(),
        my_simplex,
        m_function->get_function_simplex_type());

    return get_gradient_sum(simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
        coface_tuples,
        m_function->get_function_simplex_type()));
}
Eigen::MatrixXd LocallyDifferentiableFunction::get_local_hessian(const Simplex& my_simplex) const
{
    std::vector<Tuple> coface_tuples = simplex::upper_level_cofaces_tuples(
        m_function->mesh(),
        my_simplex,
        m_function->get_function_simplex_type());

    return get_hessian_sum(simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
        coface_tuples,
        m_function->get_function_simplex_type()));
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
    Eigen::VectorXd g = Eigen::VectorXd::Zero(m_function->embedded_dimension());
    for (const Simplex& cell : coface_simplices) {
        g += m_function->get_gradient(cell);
    }
    return g;
}
Eigen::MatrixXd LocallyDifferentiableFunction::get_hessian_sum(
    const std::vector<Simplex>& coface_simplices) const
{
    Eigen::MatrixXd h =
        Eigen::MatrixXd::Zero(m_function->embedded_dimension(), m_function->embedded_dimension());
    for (const Simplex& cell : coface_simplices) {
        h += m_function->get_hessian(cell);
    }
    return h;
}

} // namespace function

} // namespace wmtk