#include "PerSimplexDifferentiableFunction.hpp"

namespace wmtk::function {
PerSimplexDifferentiableFunction::PerSimplexDifferentiableFunction(
    const Mesh& mesh,
    PrimitiveType domain_simplex_type,
    const attribute::MeshAttributeHandle<double>& variable_attribute_handle)
    : PerSimplexFunction(mesh, domain_simplex_type)
    , m_coordinate_attribute_handle(variable_attribute_handle)
{}

PerSimplexDifferentiableFunction::~PerSimplexDifferentiableFunction() = default;

MeshAttributeHandle<double> PerSimplexDifferentiableFunction::get_coordinate_attribute_handle()
    const
{
    return m_coordinate_attribute_handle;
}

PrimitiveType PerSimplexDifferentiableFunction::get_coordinate_attribute_primitive_type() const
{
    return m_coordinate_attribute_handle.primitive_type();
}

Eigen::VectorXd PerSimplexDifferentiableFunction::get_gradient_sum(
    const std::vector<Simplex>& domain_simplices,
    const Simplex& variable_simplex) const
{
    Eigen::VectorXd g = Eigen::VectorXd::Zero(embedded_dimension());
    assert(variable_simplex.primitive_type() == get_coordinate_attribute_primitive_type());
    for (const Simplex& cell : domain_simplices) {
        assert(cell.primitive_type() == get_domain_simplex_type());
        g += get_gradient(cell, variable_simplex);
    }
    return g;
}
Eigen::MatrixXd PerSimplexDifferentiableFunction::get_hessian_sum(
    const std::vector<Simplex>& domain_simplices,
    const Simplex& variable_simplex) const
{
    assert(variable_simplex.primitive_type() == get_coordinate_attribute_primitive_type());
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(embedded_dimension(), embedded_dimension());
    for (const Simplex& cell : domain_simplices) {
        assert(cell.primitive_type() == get_domain_simplex_type());
        h += get_hessian(cell, variable_simplex);
    }
    return h;
}

Eigen::VectorXd PerSimplexDifferentiableFunction::get_gradient_sum(
    const std::vector<Tuple>& domain_simplices,
    const Tuple& variable_tuple) const
{
    const Simplex variable_simplex = as_coordinate_attribute_simplex(variable_tuple);
    Eigen::VectorXd g = Eigen::VectorXd::Zero(embedded_dimension());
    for (const Tuple& cell : domain_simplices) {
        g += get_gradient(as_domain_simplex(cell), variable_simplex);
    }
    return g;
}
Eigen::MatrixXd PerSimplexDifferentiableFunction::get_hessian_sum(
    const std::vector<Tuple>& domain_simplices,
    const Tuple& variable_tuple) const
{
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(embedded_dimension(), embedded_dimension());
    const Simplex variable_simplex = as_coordinate_attribute_simplex(variable_tuple);
    for (const Tuple& cell : domain_simplices) {
        h += get_hessian(as_domain_simplex(cell), variable_simplex);
    }
    return h;
}

long PerSimplexDifferentiableFunction::embedded_dimension() const
{
    return mesh().get_attribute_dimension(get_coordinate_attribute_handle());
}

Simplex PerSimplexDifferentiableFunction::as_coordinate_attribute_simplex(const Tuple& t) const
{
    return Simplex(get_coordinate_attribute_primitive_type(), t);
}

} // namespace wmtk::function
