#include "LocalNeighborsSumFunction.hpp"

#include "PerSimplexFunction.hpp"

#include <wmtk/simplex/cofaces_single_dimension.hpp>


namespace wmtk::function {


LocalNeighborsSumFunction::LocalNeighborsSumFunction(
    Mesh& mesh,
    const attribute::MeshAttributeHandle& handle,
    PerSimplexFunction& function)
    : Function(mesh, handle)
    , m_function(function)
{
    m_domain_simplex_type = mesh.top_simplex_type();
}

std::vector<simplex::Simplex> LocalNeighborsSumFunction::domain(
    const simplex::Simplex& variable_simplex) const
{
    return wmtk::simplex::cofaces_single_dimension_simplices(
        mesh(),
        variable_simplex,
        m_domain_simplex_type);
}

double LocalNeighborsSumFunction::get_value(const simplex::Simplex& variable_simplex) const
{
    const auto neighs = domain(variable_simplex);
    assert(variable_simplex.primitive_type() == attribute_type());

    assert(embedded_dimension() == m_function.embedded_dimension());
    assert(mesh() == m_function.mesh());

    double res = 0;
    for (const simplex::Simplex& cell : neighs) {
        assert(cell.primitive_type() == m_domain_simplex_type);
        res += m_function.get_value(cell);
    }

    return res;
}

Eigen::VectorXd LocalNeighborsSumFunction::get_gradient(
    const simplex::Simplex& variable_simplex) const
{
    const auto neighs = domain(variable_simplex);
    assert(variable_simplex.primitive_type() == attribute_type());
    assert(embedded_dimension() == m_function.embedded_dimension());
    assert(mesh() == m_function.mesh());

    Eigen::VectorXd res = Eigen::VectorXd::Zero(embedded_dimension());

    for (const simplex::Simplex& cell : neighs) {
        assert(cell.primitive_type() == m_domain_simplex_type);
        res += m_function.get_gradient(cell, variable_simplex);
    }

    return res;
}

Eigen::MatrixXd LocalNeighborsSumFunction::get_hessian(
    const simplex::Simplex& variable_simplex) const
{
    const auto neighs = domain(variable_simplex);
    assert(variable_simplex.primitive_type() == attribute_type());
    assert(embedded_dimension() == m_function.embedded_dimension());
    assert(mesh() == m_function.mesh());

    Eigen::MatrixXd res = Eigen::MatrixXd::Zero(embedded_dimension(), embedded_dimension());

    for (const simplex::Simplex& cell : neighs) {
        assert(cell.primitive_type() == m_domain_simplex_type);
        res += m_function.get_hessian(cell, variable_simplex);
    }

    return res;
}

} // namespace wmtk::function
