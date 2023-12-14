#include "LocalNeighborsSumFunction.hpp"

#include "AsdPerSimplexFunction.hpp"

#include <wmtk/simplex/cofaces_single_dimension.hpp>


namespace wmtk::function {

std::vector<simplex::Simplex> LocalNeighborsSumFunction::get_local_neighborhood_domain_simplices(
    const simplex::Simplex& variable_simplex) const
{
    return wmtk::simplex::cofaces_single_dimension_simplices(
        mesh(),
        variable_simplex,
        m_domain_simplex_type);
}

double LocalNeighborsSumFunction::get_value(const simplex::Simplex& variable_simplex) const
{
    auto neighs = get_local_neighborhood_domain_simplices(variable_simplex);
    assert(variable_simplex.primitive_type() == attribute_type());

    double res = 0;
    for (const Simplex& cell : neighs) {
        assert(cell.primitive_type() == m_domain_simplex_type);
        res += m_function->get_value(cell);
    }

    return res;
}

Eigen::VectorXd LocalNeighborsSumFunction::get_gradient(
    const simplex::Simplex& variable_simplex) const
{
    auto neighs = get_local_neighborhood_domain_simplices(variable_simplex);
    assert(variable_simplex.primitive_type() == attribute_type());

    Eigen::VectorXd res = Eigen::VectorXd::Zero(embedded_dimension());

    for (const Simplex& cell : neighs) {
        assert(cell.primitive_type() == m_domain_simplex_type);
        res += m_function->get_gradient(cell, variable_simplex);
    }

    return res;
}

Eigen::MatrixXd LocalNeighborsSumFunction::get_hessian(
    const simplex::Simplex& variable_simplex) const
{
    auto neighs = get_local_neighborhood_domain_simplices(variable_simplex);
    assert(variable_simplex.primitive_type() == attribute_type());

    Eigen::MatrixXd res = Eigen::MatrixXd::Zero(embedded_dimension(), embedded_dimension());

    for (const Simplex& cell : neighs) {
        assert(cell.primitive_type() == m_domain_simplex_type);
        res += m_function->get_hessian(cell, variable_simplex);
    }

    return res;
}

} // namespace wmtk::function